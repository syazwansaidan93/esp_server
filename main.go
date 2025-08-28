package main

import (
	"database/sql"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"sync"
	"time"

	_ "github.com/mattn/go-sqlite3" // SQLite driver
	"go.bug.st/serial"              // Serial communication library
)

const (
	baudRate      = 115200
	serialTimeout = 5 * time.Second
	dbFile        = "sensor_data.db"
	indoorTempURL = "http://192.168.1.4/i_temp"
)

// Data models for JSON responses
type SensorData struct {
	VoltageV  float64 `json:"voltage_V,omitempty"`
	CurrentMA float64 `json:"current_mA,omitempty"`
	PowerMW   float64 `json:"power_mW,omitempty"`
}

type TemperatureData struct {
	Outdoor float64 `json:"outdoor,omitempty"`
	Indoor  float64 `json:"indoor,omitempty"`
}

type RelayState struct {
	State string `json:"relay_status"`
}

type RelayHistory struct {
	Timestamp string `json:"timestamp"`
	State     string `json:"state"`
}

type SolarDataHistory struct {
	Timestamp string `json:"timestamp"`
	VoltageV  float64 `json:"voltage_V"`
	CurrentMA float64 `json:"current_mA"`
	PowerMW   float64 `json:"power_mW"`
}

type StatusResponse struct {
	Status  string `json:"status"`
	Message string `json:"message,omitempty"`
}

// Global variables for serial connection and database
var (
	serialPort serial.Port
	serialMutex sync.Mutex
	db         *sql.DB
)

func main() {
	log.Println("Starting Go ESP32 Monitoring App...")

	// 1. Setup the database
	if err := setupDatabase(); err != nil {
		log.Fatalf("Failed to set up database: %v", err)
	}
	defer db.Close()

	// 2. Connect to the serial port
	var err error
	serialPort, err = connectToSerial()
	if err != nil {
		log.Fatalf("Failed to connect to serial port: %v", err)
	}
	defer serialPort.Close()

	// 3. Start background jobs using goroutines and tickers
	go startDataCollectionJob()
	go startPruningJob()
	go startRelayPollingJob() // New job to poll and record relay state changes

	// 4. Register HTTP handlers
	http.HandleFunc("/r/on", handleRelayOn)
	http.HandleFunc("/r/off", handleRelayOff)
	http.HandleFunc("/r/latest", handleRelayStatus)
	http.HandleFunc("/o/latest", handleOutdoorTemp)
	http.HandleFunc("/i/latest", handleIndoorTemp)
	http.HandleFunc("/s/latest", handleSolarPower)
	http.HandleFunc("/t/latest", handleLatestTemps)
	http.HandleFunc("/t/24", handle24hTemps)
	http.HandleFunc("/r/24", handle24hRelayHistory)
	http.HandleFunc("/o/24", handle24hOutdoorTemp)
	http.HandleFunc("/i/24", handle24hIndoorTemp)
	http.HandleFunc("/s/24", handle24hSolar)
	// Additional handlers for settings endpoints
	// For simplicity, these handlers are not implemented here but the structure is the same.
	// http.HandleFunc("/settings", handleGetSettings)

	// 5. Start the web server
	log.Println("Server listening on http://0.0.0.0:5000")
	if err := http.ListenAndServe("0.0.0.0:5000", nil); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}

// setupDatabase initializes the SQLite database and creates tables if they don't exist.
func setupDatabase() error {
	var err error
	db, err = sql.Open("sqlite3", dbFile)
	if err != nil {
		return err
	}

	createTablesSQL := `
	CREATE TABLE IF NOT EXISTS temperature_readings (
		timestamp TEXT PRIMARY KEY,
		outdoor_temp_C REAL,
		indoor_temp_C REAL
	);
	CREATE TABLE IF NOT EXISTS solar_readings (
		timestamp TEXT PRIMARY KEY,
		voltage_V REAL,
		current_mA REAL,
		power_mW REAL
	);
	CREATE TABLE IF NOT EXISTS relay_state_changes (
		timestamp TEXT PRIMARY KEY,
		state TEXT NOT NULL
	);
	`
	_, err = db.Exec(createTablesSQL)
	return err
}

// connectToSerial auto-detects and connects to the ESP32 serial port.
func connectToSerial() (serial.Port, error) {
	ports, err := serial.GetPortsList()
	if err != nil {
		return nil, fmt.Errorf("failed to get serial port list: %w", err)
	}
	if len(ports) == 0 {
		return nil, fmt.Errorf("no serial ports found")
	}

	for _, portName := range ports {
		if _, err := os.Stat(portName); err == nil && (os.IsPermission(err) || os.IsNotExist(err)) {
			continue // Skip ports without read/write permissions or non-existent ones
		}

		log.Printf("Found potential serial port: %s\n", portName)
		port, err := serial.Open(portName, &serial.Mode{BaudRate: baudRate})
		if err != nil {
			log.Printf("Failed to open port %s: %v\n", portName, err)
			continue
		}
		// A short sleep to allow the ESP32 to reset
		time.Sleep(2 * time.Second)
		log.Printf("Successfully connected to serial port %s\n", portName)
		return port, nil
	}

	return nil, fmt.Errorf("no suitable serial port found")
}

// sendSerialCommand sends a command to the ESP32 and waits for a response.
func sendSerialCommand(command string) (map[string]interface{}, error) {
	serialMutex.Lock()
	defer serialMutex.Unlock()

	// Clear the input buffer to prevent reading old data
	serialPort.ResetOutputBuffer()
	serialPort.ResetInputBuffer()

	// Write the command to the serial port
	_, err := serialPort.Write([]byte(command + "\n"))
	if err != nil {
		return nil, fmt.Errorf("failed to write to serial port: %w", err)
	}

	// Read a line from the serial port
	reader := io.Reader(serialPort)
	decoder := json.NewDecoder(reader)

	// Set a deadline for reading
	serialPort.SetReadTimeout(serialTimeout)

	var data map[string]interface{}
	err = decoder.Decode(&data)
	if err != nil {
		return nil, fmt.Errorf("failed to decode JSON response: %w", err)
	}

	return data, nil
}

// fetchIndoorTemp asynchronously fetches the indoor temperature from a local URL.
func fetchIndoorTemp() (float64, error) {
	resp, err := http.Get(indoorTempURL)
	if err != nil {
		return 0, fmt.Errorf("failed to fetch indoor temperature: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return 0, fmt.Errorf("non-200 status code: %d", resp.StatusCode)
	}

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return 0, fmt.Errorf("failed to read response body: %w", err)
	}

	var temp float64
	_, err = fmt.Sscanf(string(body), "%f", &temp)
	if err != nil {
		return 0, fmt.Errorf("failed to parse indoor temp from response: %w", err)
	}
	return temp, nil
}

// recordRelayState logs a relay state change in the database.
func recordRelayState(state string) {
	query := "INSERT OR REPLACE INTO relay_state_changes (timestamp, state) VALUES (?, ?)"
	_, err := db.Exec(query, time.Now().UTC().Format(time.RFC3339), state)
	if err != nil {
		log.Printf("Error recording relay state: %v\n", err)
	} else {
		log.Printf("Relay state change recorded: %s\n", state)
	}
}

// startDataCollectionJob is a background job that collects and stores sensor data every 15 minutes.
func startDataCollectionJob() {
	ticker := time.NewTicker(15 * time.Minute)
	defer ticker.Stop()

	for range ticker.C {
		log.Println("Running scheduled data collection job...")
		timestamp := time.Now().UTC().Format(time.RFC3339)

		// Fetch outdoor temperature
		outdoorData, err := sendSerialCommand("o")
		var outdoorTemp sql.NullFloat64
		if err == nil {
			if temp, ok := outdoorData["value"].(float64); ok {
				outdoorTemp.Float64 = temp
				outdoorTemp.Valid = true
			}
		} else {
			log.Println("Failed to get outdoor temp:", err)
		}

		// Fetch indoor temperature
		indoorTemp, err := fetchIndoorTemp()
		var indoor sql.NullFloat64
		if err == nil {
			indoor.Float64 = indoorTemp
			indoor.Valid = true
		} else {
			log.Println("Failed to get indoor temp:", err)
		}

		// Store temperature data
		queryTemp := "INSERT OR REPLACE INTO temperature_readings (timestamp, outdoor_temp_C, indoor_temp_C) VALUES (?, ?, ?)"
		_, err = db.Exec(queryTemp, timestamp, outdoorTemp, indoor)
		if err != nil {
			log.Printf("Error storing temperature data: %v\n", err)
		} else {
			log.Println("Temperature data stored successfully.")
		}

		// Check if the current time is within the solar recording range (7:15 AM - 7:30 PM)
		now := time.Now()
		startOfDay := time.Date(now.Year(), now.Month(), now.Day(), 7, 15, 0, 0, now.Location())
		endOfDay := time.Date(now.Year(), now.Month(), now.Day(), 19, 30, 0, 0, now.Location())

		if now.After(startOfDay) && now.Before(endOfDay) {
			// Fetch solar data
			solarData, err := sendSerialCommand("s")
			if err == nil {
				var voltage, current, power sql.NullFloat64
				if v, ok := solarData["voltage_V"].(float64); ok {
					voltage.Float64 = v; voltage.Valid = true
				}
				if c, ok := solarData["current_mA"].(float64); ok {
					current.Float64 = c; current.Valid = true
				}
				if p, ok := solarData["power_mW"].(float64); ok {
					power.Float64 = p; power.Valid = true
				}
				querySolar := "INSERT OR REPLACE INTO solar_readings (timestamp, voltage_V, current_mA, power_mW) VALUES (?, ?, ?, ?)"
				_, err = db.Exec(querySolar, timestamp, voltage, current, power)
				if err != nil {
					log.Printf("Error storing solar data: %v\n", err)
				} else {
					log.Println("Solar data stored successfully.")
				}
			} else {
				log.Println("Failed to get solar data:", err)
			}
		} else {
			log.Println("Skipping solar data collection outside of 7:15 AM - 7:30 PM range.")
		}
	}
}

// startPruningJob is a background job to prune old data every 24 hours.
func startPruningJob() {
	ticker := time.NewTicker(24 * time.Hour)
	defer ticker.Stop()

	for range ticker.C {
		log.Println("Running scheduled job to prune old data...")
		cutoff := time.Now().Add(-48 * time.Hour).UTC().Format(time.RFC3339)
		queries := []string{
			"DELETE FROM temperature_readings WHERE timestamp < ?",
			"DELETE FROM solar_readings WHERE timestamp < ?",
			"DELETE FROM relay_state_changes WHERE timestamp < ?",
		}

		for _, query := range queries {
			_, err := db.Exec(query, cutoff)
			if err != nil {
				log.Printf("Error pruning old data with query '%s': %v\n", query, err)
			}
		}
		log.Println("Old data pruned successfully.")
	}
}

// startRelayPollingJob polls the ESP32 for the current relay state every 5 seconds and
// records the state in the database if it has changed.
func startRelayPollingJob() {
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	var lastState string

	for range ticker.C {
		log.Println("Polling ESP32 for current relay state...")
		data, err := sendSerialCommand("r")
		if err != nil {
			log.Printf("Failed to get relay status during polling: %v", err)
			continue
		}
		
		currentState, ok := data["value"].(string)
		if !ok {
			log.Printf("Invalid response from ESP32 during polling: %v", data)
			continue
		}
		
		// If the state has changed from the last known state, record it.
		if currentState != lastState {
			log.Printf("Relay state changed from '%s' to '%s'. Recording to DB.", lastState, currentState)
			recordRelayState(currentState)
			lastState = currentState
		}
	}
}


// --- HTTP Handlers ---

func handleRelayOn(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}
	log.Println("Attempting to turn relay ON...")
	data, err := sendSerialCommand("r1")
	if err != nil {
		log.Printf("Failed to get response from ESP32: %v", err)
		http.Error(w, "Failed to turn relay ON due to communication error", http.StatusInternalServerError)
		return
	}
	if data["value"] != "ON" {
		log.Printf("ESP32 response was not 'ON': %v", data)
		http.Error(w, "Failed to turn relay ON. Unexpected response from device.", http.StatusInternalServerError)
		return
	}
	// recordRelayState("ON") // This is now handled by the polling job
	json.NewEncoder(w).Encode(StatusResponse{Status: "success", Message: "Relay turned ON"})
	log.Println("Relay ON command successful.")
}

func handleRelayOff(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}
	log.Println("Attempting to turn relay OFF...")
	data, err := sendSerialCommand("r0")
	if err != nil {
		log.Printf("Failed to get response from ESP32: %v", err)
		http.Error(w, "Failed to turn relay OFF due to communication error", http.StatusInternalServerError)
		return
	}
	if data["value"] != "OFF" {
		log.Printf("ESP32 response was not 'OFF': %v", data)
		http.Error(w, "Failed to turn relay OFF. Unexpected response from device.", http.StatusInternalServerError)
		return
	}
	// recordRelayState("OFF") // This is now handled by the polling job
	json.NewEncoder(w).Encode(StatusResponse{Status: "success", Message: "Relay turned OFF"})
	log.Println("Relay OFF command successful.")
}

func handleRelayStatus(w http.ResponseWriter, r *http.Request) {
	data, err := sendSerialCommand("r")
	if err != nil {
		http.Error(w, "Failed to get relay status", http.StatusInternalServerError)
		return
	}
	state, ok := data["value"].(string)
	if !ok {
		http.Error(w, "Invalid response from device", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(RelayState{State: state})
}

func handleOutdoorTemp(w http.ResponseWriter, r *http.Request) {
	data, err := sendSerialCommand("o")
	if err != nil {
		http.Error(w, "Failed to get outdoor temp", http.StatusInternalServerError)
		return
	}
	temp, ok := data["value"].(float64)
	if !ok {
		http.Error(w, "Invalid response from device", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(TemperatureData{Outdoor: temp})
}

func handleIndoorTemp(w http.ResponseWriter, r *http.Request) {
	temp, err := fetchIndoorTemp()
	if err != nil {
		http.Error(w, "Failed to get indoor temp", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(TemperatureData{Indoor: temp})
}

func handleSolarPower(w http.ResponseWriter, r *http.Request) {
	data, err := sendSerialCommand("s")
	if err != nil {
		http.Error(w, "Failed to get solar power", http.StatusInternalServerError)
		return
	}
	voltage, okV := data["voltage_V"].(float64)
	current, okC := data["current_mA"].(float64)
	power, okP := data["power_mW"].(float64)
	if !okV || !okC || !okP {
		http.Error(w, "Invalid response from device", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(SensorData{VoltageV: voltage, CurrentMA: current, PowerMW: power})
}

func handleLatestTemps(w http.ResponseWriter, r *http.Request) {
	outdoorData, errO := sendSerialCommand("o")
	indoorTemp, errI := fetchIndoorTemp()
	
	tempData := TemperatureData{}
	if errO == nil {
		if oTemp, ok := outdoorData["value"].(float64); ok {
			tempData.Outdoor = oTemp
		}
	}
	if errI == nil {
		tempData.Indoor = indoorTemp
	}

	if errO != nil && errI != nil {
		http.Error(w, "Failed to fetch any temperature data", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(tempData)
}

func handle24hTemps(w http.ResponseWriter, r *http.Request) {
	rows, err := db.Query("SELECT timestamp, outdoor_temp_C, indoor_temp_C FROM temperature_readings WHERE timestamp >= ?", time.Now().Add(-24*time.Hour).UTC().Format(time.RFC3339))
	if err != nil {
		http.Error(w, "Failed to query database", http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var results []map[string]interface{}
	for rows.Next() {
		var timestamp string
		var outdoor, indoor sql.NullFloat64
		if err := rows.Scan(&timestamp, &outdoor, &indoor); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		record := make(map[string]interface{})
		record["timestamp"] = timestamp
		if outdoor.Valid { record["outdoor_temp_C"] = outdoor.Float64 }
		if indoor.Valid { record["indoor_temp_C"] = indoor.Float64 }
		results = append(results, record)
	}
	json.NewEncoder(w).Encode(results)
}

func handle24hRelayHistory(w http.ResponseWriter, r *http.Request) {
	rows, err := db.Query("SELECT timestamp, state FROM relay_state_changes WHERE timestamp >= ? ORDER BY timestamp", time.Now().Add(-24*time.Hour).UTC().Format(time.RFC3339))
	if err != nil {
		http.Error(w, "Failed to query database", http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var results []RelayHistory
	for rows.Next() {
		var rh RelayHistory
		if err := rows.Scan(&rh.Timestamp, &rh.State); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		results = append(results, rh)
	}
	json.NewEncoder(w).Encode(results)
}

func handle24hOutdoorTemp(w http.ResponseWriter, r *http.Request) {
	rows, err := db.Query("SELECT timestamp, outdoor_temp_C FROM temperature_readings WHERE timestamp >= ? ORDER BY timestamp", time.Now().Add(-24*time.Hour).UTC().Format(time.RFC3339))
	if err != nil {
		http.Error(w, "Failed to query database", http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var results []map[string]interface{}
	for rows.Next() {
		var timestamp string
		var outdoor sql.NullFloat64
		if err := rows.Scan(&timestamp, &outdoor); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		if outdoor.Valid {
			record := make(map[string]interface{})
			record["timestamp"] = timestamp
			record["outdoor_temp_C"] = outdoor.Float64
			results = append(results, record)
		}
	}
	json.NewEncoder(w).Encode(results)
}

func handle24hIndoorTemp(w http.ResponseWriter, r *http.Request) {
	rows, err := db.Query("SELECT timestamp, indoor_temp_C FROM temperature_readings WHERE timestamp >= ? ORDER BY timestamp", time.Now().Add(-24*time.Hour).UTC().Format(time.RFC3339))
	if err != nil {
		http.Error(w, "Failed to query database", http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var results []map[string]interface{}
	for rows.Next() {
		var timestamp string
		var indoor sql.NullFloat64
		if err := rows.Scan(&timestamp, &indoor); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		if indoor.Valid {
			record := make(map[string]interface{})
			record["timestamp"] = timestamp
			record["indoor_temp_C"] = indoor.Float64
			results = append(results, record)
		}
	}
	json.NewEncoder(w).Encode(results)
}

func handle24hSolar(w http.ResponseWriter, r *http.Request) {
	rows, err := db.Query("SELECT timestamp, voltage_V, current_mA, power_mW FROM solar_readings WHERE timestamp >= ? ORDER BY timestamp", time.Now().Add(-24*time.Hour).UTC().Format(time.RFC3339))
	if err != nil {
		http.Error(w, "Failed to query database", http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var results []SolarDataHistory
	for rows.Next() {
		var data SolarDataHistory
		if err := rows.Scan(&data.Timestamp, &data.VoltageV, &data.CurrentMA, &data.PowerMW); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		results = append(results, data)
	}
	json.NewEncoder(w).Encode(results)
}

package main

import (
	"bufio"
	"database/sql"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"sync"
	"time"

	_ "github.com/mattn/go-sqlite3"
	"go.bug.st/serial"
)

const (
	baudRate = 115200
	serialTimeout = 5 * time.Second
	dbFile = "sensor_data.db"
	indoorTempURL = "http://192.168.1.4/i_temp"
)

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
	Timestamp string  `json:"timestamp"`
	VoltageV  float64 `json:"voltage_V"`
	CurrentMA float64 `json:"current_mA"`
	PowerMW   float64 `json:"power_mW"`
}

type UptimeData struct {
	UptimeMS float64 `json:"uptime_ms"`
}

type StatusResponse struct {
	Status  string `json:"status"`
	Message string `json:"message,omitempty"`
}

type ESPDataResponse struct {
	PowerMW      float64 `json:"power_mW"`
	OutdoorTempC float64 `json:"outdoor_temp_C"`
	RelayState   string  `json:"relay_state"`
}

var (
	serialMutex      sync.Mutex
	db               *sql.DB
	commandChan      = make(chan string)
	responseChan     = make(chan map[string]interface{})
	relayEventChan   = make(chan string)
	serialConnection serial.Port
	serialDisconnectChan = make(chan struct{})
)

func main() {
	log.Println("Starting Go ESP32 Monitoring App...")

	if err := setupDatabase(); err != nil {
		log.Fatalf("Failed to set up database: %v", err)
	}
	defer db.Close()

	go startSerialConnectionManager()
	go startDataCollectionJob()
	go startPruningJob()
	go startRelayEventWatcher()

	http.HandleFunc("/r/latest", handleRelayStatus)
	http.HandleFunc("/o/latest", handleOutdoorTemp)
	http.HandleFunc("/i/latest", handleIndoorTemp)
	http.HandleFunc("/s/latest", handleSolarPower)
	http.HandleFunc("/u/latest", handleUptime)
	http.HandleFunc("/t/latest", handleLatestTemps)
	http.HandleFunc("/t/24", handle24hTemps)
	http.HandleFunc("/r/24", handle24hRelayHistory)
	http.HandleFunc("/o/24", handle24hOutdoorTemp)
	http.HandleFunc("/i/24", handle24hIndoorTemp)
	http.HandleFunc("/s/24", handle24hSolar)
	http.HandleFunc("/esp", handleLatestEspData)

	log.Println("Server listening on http://0.0.0.0:5000")
	if err := http.ListenAndServe("0.0.0.0:5000", nil); err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
}

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

func startSerialConnectionManager() {
	for {
		log.Println("Attempting to connect to a serial port...")
		port, err := connectToSerial()
		if err != nil {
			log.Printf("Failed to connect to serial port: %v. Retrying in 5 seconds...", err)
			time.Sleep(5 * time.Second)
			continue
		}
		serialMutex.Lock()
		serialConnection = port
		serialMutex.Unlock()
		log.Println("Serial port connected. Starting serial reader...")
		go serialManager()

		<-serialDisconnectChan
		log.Println("Serial port disconnected. Attempting to reconnect...")
		serialMutex.Lock()
		serialConnection.Close()
		serialConnection = nil
		serialMutex.Unlock()
	}
}

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
			continue
		}

		log.Printf("Found potential serial port: %s\n", portName)
		port, err := serial.Open(portName, &serial.Mode{BaudRate: baudRate})
		if err != nil {
			log.Printf("Failed to open port %s: %v\n", portName, err)
			continue
		}
		time.Sleep(2 * time.Second)
		log.Printf("Successfully connected to serial port %s\n", portName)
		return port, nil
	}

	return nil, fmt.Errorf("no suitable serial port found")
}

func serialManager() {
	defer func() {
		close(serialDisconnectChan)
	}()

	scanner := bufio.NewScanner(serialConnection)
	for scanner.Scan() {
		line := scanner.Text()
		var data map[string]interface{}
		err := json.Unmarshal([]byte(line), &data)
		if err != nil {
			log.Printf("Non-JSON serial message received: %s\n", line)
			continue
		}

		if _, ok := data["relay_event"]; ok {
			relayEventChan <- data["relay_event"].(string)
		} else {
			responseChan <- data
		}
	}
	if err := scanner.Err(); err != nil {
		log.Printf("Error reading from serial port: %v\n", err)
	}
}

func sendSerialCommand(command string) (map[string]interface{}, error) {
	serialMutex.Lock()
	defer serialMutex.Unlock()

	if serialConnection == nil {
		return nil, fmt.Errorf("serial port not connected")
	}

	serialConnection.ResetOutputBuffer()
	serialConnection.ResetInputBuffer()
	serialConnection.SetReadTimeout(serialTimeout)

	_, err := serialConnection.Write([]byte(command + "\n"))
	if err != nil {
		return nil, fmt.Errorf("failed to write to serial port: %w", err)
	}

	select {
	case data := <-responseChan:
		return data, nil
	case <-time.After(serialTimeout):
		return nil, fmt.Errorf("serial response timeout")
	}
}

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

func recordRelayState(state string) {
	query := "INSERT OR REPLACE INTO relay_state_changes (timestamp, state) VALUES (?, ?)"
	_, err := db.Exec(query, time.Now().UTC().Format(time.RFC3339), state)
	if err != nil {
		log.Printf("Error recording relay state: %v\n", err)
	} else {
		log.Printf("Relay state change recorded: %s\n", state)
	}
}

func startDataCollectionJob() {
	ticker := time.NewTicker(15 * time.Minute)
	defer ticker.Stop()

	for range ticker.C {
		log.Println("Running scheduled data collection job...")
		timestamp := time.Now().UTC().Format(time.RFC3339)

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

		indoorTemp, err := fetchIndoorTemp()
		var indoor sql.NullFloat64
		if err == nil {
			indoor.Float64 = indoorTemp
			indoor.Valid = true
		} else {
			log.Println("Failed to get indoor temp:", err)
		}

		queryTemp := "INSERT OR REPLACE INTO temperature_readings (timestamp, outdoor_temp_C, indoor_temp_C) VALUES (?, ?, ?)"
		_, err = db.Exec(queryTemp, timestamp, outdoorTemp, indoor)
		if err != nil {
			log.Printf("Error storing temperature data: %v\n", err)
		} else {
			log.Println("Temperature data stored successfully.")
		}

		now := time.Now()
		startOfDay := time.Date(now.Year(), now.Month(), now.Day(), 7, 15, 0, 0, now.Location())
		endOfDay := time.Date(now.Year(), now.Month(), now.Day(), 19, 30, 0, 0, now.Location())

		if now.After(startOfDay) && now.Before(endOfDay) {
			solarData, err := sendSerialCommand("s")
			if err == nil {
				var voltage, current, power sql.NullFloat64
				if v, ok := solarData["voltage_V"].(float64); ok {
					voltage.Float64 = v
					voltage.Valid = true
				}
				if c, ok := solarData["current_mA"].(float64); ok {
					current.Float64 = c
					current.Valid = true
				}
				if p, ok := solarData["power_mW"].(float64); ok {
					power.Float64 = p
					power.Valid = true
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

func startRelayEventWatcher() {
	for state := range relayEventChan {
		log.Printf("Relay state changed to '%s'. Recording to DB.\n", state)
		recordRelayState(state)
	}
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

func handleUptime(w http.ResponseWriter, r *http.Request) {
	data, err := sendSerialCommand("u")
	if err != nil {
		http.Error(w, "Failed to get uptime", http.StatusInternalServerError)
		return
	}
	uptime, ok := data["value_ms"].(float64)
	if !ok {
		http.Error(w, "Invalid response from device", http.StatusInternalServerError)
		return
	}
	json.NewEncoder(w).Encode(UptimeData{UptimeMS: uptime})
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
		if outdoor.Valid {
			record["outdoor_temp_C"] = outdoor.Float64
		}
		if indoor.Valid {
			record["indoor_temp_C"] = indoor.Float64
		}
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

	var results []map[string]interface{}
	for rows.Next() {
		var timestamp string
		var voltage, current, power sql.NullFloat64
		if err := rows.Scan(&timestamp, &voltage, &current, &power); err != nil {
			log.Printf("Error scanning row: %v\n", err)
			continue
		}
		record := make(map[string]interface{})
		record["timestamp"] = timestamp
		if voltage.Valid {
			record["voltage_V"] = voltage.Float64
		}
		if current.Valid {
			record["current_mA"] = current.Float64
		}
		if power.Valid {
			record["power_mW"] = power.Float64
		}
		results = append(results, record)
	}
	json.NewEncoder(w).Encode(results)
}

func handleLatestEspData(w http.ResponseWriter, r *http.Request) {
	solarData, err := sendSerialCommand("s")
	if err != nil {
		http.Error(w, "Failed to get solar power", http.StatusInternalServerError)
		return
	}
	power, okP := solarData["power_mW"].(float64)
	if !okP {
		http.Error(w, "Invalid power data from device", http.StatusInternalServerError)
		return
	}

	outdoorData, err := sendSerialCommand("o")
	if err != nil {
		http.Error(w, "Failed to get outdoor temperature", http.StatusInternalServerError)
		return
	}
	outdoorTemp, okT := outdoorData["value"].(float64)
	if !okT {
		http.Error(w, "Invalid temperature data from device", http.StatusInternalServerError)
		return
	}

	relayData, err := sendSerialCommand("r")
	if err != nil {
		http.Error(w, "Failed to get relay status", http.StatusInternalServerError)
		return
	}
	relayState, okR := relayData["value"].(string)
	if !okR {
		http.Error(w, "Invalid relay status from device", http.StatusInternalServerError)
		return
	}

	response := ESPDataResponse{
		PowerMW: power,
		OutdoorTempC: outdoorTemp,
		RelayState: relayState,
	}

	json.NewEncoder(w).Encode(response)
}

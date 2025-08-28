# Go ESP32 Monitoring Application

This application is a backend service written in Go for monitoring an ESP32C3-based sensor system. It communicates with the ESP32 via a serial connection, collects sensor data, and stores it in a local SQLite database. The application also exposes a simple HTTP API for retrieving the data and controlling a relay.

## Features

- **Serial Communication:** Establishes a connection with an ESP32C3 to send commands and receive sensor data in JSON format.
- **Sensor Data Collection:**
  - **Temperature:** Records both outdoor temperature (from the ESP32) and indoor temperature (from a separate HTTP endpoint) every 15 minutes.
  - **Solar Power:** Records solar panel voltage, current, and power at 15-minute intervals, but **only between 7:15 AM and 7:30 PM**.
- **Local Database:** Uses SQLite to store all sensor readings and relay state changes.
- **Scheduled Pruning:** Automatically deletes data older than 48 hours every 24 hours to manage disk space.
- **Relay State Logging:** A background job **polls the ESP32 every 5 seconds** and automatically records a new entry in the database whenever the relay's state changes. This ensures a complete history of all changes, regardless of how they were initiated.
- **HTTP API:** Provides various endpoints to fetch the latest sensor values, get historical data, and control a connected relay.

## Prerequisites

- Go (version 1.18 or higher)
- An ESP32C3 device running the appropriate firmware for serial communication.
- A system with a serial port, such as an Orange Pi Zero 3 with Armbian.

## Installation and Usage

1.  **Clone the repository:**
    ```sh
    git clone <repository_url>
    cd <repository_folder>
    ```

2.  **Install dependencies:**
    ```sh
    go mod tidy
    ```

3.  **Build the application:**
    ```sh
    go build -o go.esp32.monitor
    ```

4.  **Run the application:**
    ```sh
    ./go.esp32.monitor
    ```

5.  **Running as a Service (Recommended for servers like Orange Pi):**
    For a more robust setup, you can run the application as a background service using `systemd`. A sample service file is provided in the documentation for reference.

    The service file `go_esp32_monitor.service` should look like this:
    ```
    [Unit]
    Description=Go ESP32 Monitoring Application
    After=network.target

    [Service]
    ExecStart=/home/wan/esp1-go/go.esp32.monitor
    WorkingDirectory=/home/wan/
    User=wan
    Restart=always
    RestartSec=3

    [Install]
    WantedBy=multi-user.target
    ```

## Configuration

The indoor temperature sensor URL is currently hardcoded. You can change it in the `main.go` file:

```go
const (
    // ...
    indoorTempURL = "http://192.168.1.4/i_temp"
)
```

## API Endpoints

The application exposes the following HTTP endpoints on port `5000`:

| Endpoint             | Method | Description                               |
|----------------------|--------|-------------------------------------------|
| `/r/on`              | `POST`   | Turns the relay ON.                       |
| `/r/off`             | `POST`   | Turns the relay OFF.                      |
| `/r/latest`          | `GET`    | Gets the current state of the relay.      |
| `/o/latest`          | `GET`    | Gets the latest outdoor temperature.      |
| `/i/latest`          | `GET`    | Gets the latest indoor temperature.       |
| `/s/latest`          | `GET`    | Gets the latest solar power readings.     |
| `/t/latest`          | `GET`    | Gets the latest outdoor and indoor temps. |
| `/t/24`              | `GET`    | Gets temperature data for the last 24h.   |
| `/r/24`              | `GET`    | Gets relay history for the last 24h.      |
| `/o/24`              | `GET`    | Gets outdoor temperature for the last 24h.|
| `/i/24`              | `GET`    | Gets indoor temperature for the last 24h. |
| `/s/24`              | `GET`    | Gets solar power data for the last 24h.   |

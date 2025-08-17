# FirePilot Firmware

**FirePilot** is an ESP32-based flight controller firmware that provides high-rate telemetry, structured logging, and MQTT integration for drones. It handles IMU data acquisition, log streaming, and wireless communication with ground control systems.

## Features

- **Real-time Flight Telemetry** – Continuous IMU and sensor data streaming at 100 Hz
- **Structured Logging System** – Comprehensive logging over serial and MQTT protocols
- **MQTT API Integration** – Standardized topic structure for seamless ground control integration
- **Robust Wi-Fi Connectivity** – Automated network setup with intelligent reconnection handling
- **JSON Data Format** – UTF-8 encoded JSON payloads for all telemetry and command data
- **Modular Architecture** – Extensible design with configurable sinks, services, and sensor providers

## Hardware Requirements

- ESP32 development board
- MPU-9250 IMU sensor
- Wi-Fi network access
- MQTT broker (default configuration: `192.168.1.200:1883`)

## Setup and Configuration

### Wi-Fi Credentials

Configure your network settings in `src/secrets.hpp`, see config in `src/secrets.hpp.example`:

```cpp
namespace secrets
{
    const char *wifi_ssid = "YOUR_WIFI_SSID";
    const char *wifi_password = "YOUR_WIFI_PASSWORD";
    const IPAddress mqtt_broker(192, 168, 1, 200);
    const uint16_t mqtt_port = 1883;
}
```

### MQTT Broker Configuration

The firmware connects to an MQTT broker for telemetry streaming and command reception. Default broker: `192.168.1.200:1883`

For detailed MQTT configuration, topic structure, and API documentation, see `docs/mqtt.md`.

## Quick Start

1. Clone the repository and open in your preferred ESP32 development environment
2. Update Wi-Fi credentials in `src/secrets.hpp`
3. Configure MQTT broker settings if different from defaults
4. Flash the firmware to your ESP32 board
5. Connect the MPU-9250 sensor according to the wiring diagram
6. Power on and monitor serial output for connection status

## Development

FirePilot uses a modular architecture that allows for easy extension and customization:

- **Sensor Providers** – Abstract sensor interfaces for easy hardware swapping
- **Data Sinks** – Configurable output destinations (serial, MQTT, file)
- **Service Layer** – Core flight control and telemetry services
- **Communication Protocol** – JSON-based message format for interoperability

## Contributing

Contributions are welcome! Please ensure all code follows the existing style conventions and includes appropriate documentation.

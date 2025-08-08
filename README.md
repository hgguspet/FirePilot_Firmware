# FirePilot Firmware

This is an ESP32-based firmware for FirePilot drone telemetry and control system. The firmware provides IMU sensor data collection, telemetry services, and MQTT connectivity for real-time communication.

## Features

- **IMU Sensor Support**: MPU-9250 9-axis motion sensor integration
- **Telemetry System**: Real-time sensor data collection and transmission
- **MQTT Connectivity**: Wireless communication via MQTT protocol
- **JSON Encoding**: Structured data format for telemetry
- **Wi-Fi Integration**: Wireless networking capabilities

## Hardware Requirements

- ESP32 development board
- MPU-9250 IMU sensor
- Wi-Fi network access
- MQTT broker (configured to run on 192.168.1.200:1883) by default

## Setup and Configuration

### 1. Configure Wi-Fi Credentials

**Important**: Before building and uploading the firmware, you must configure your Wi-Fi credentials in the `secrets.hpp` file.

1. Navigate to `src/secrets.hpp`
2. Update the Wi-Fi SSID and password:

```cpp
namespace secrets
{
    const char *wifi_ssid = "YOUR_WIFI_SSID";
    const char *wifi_password = "YOUR_WIFI_PASSWORD";
}
```

**Security Note**: The `secrets.hpp` file contains sensitive information. Make sure to:

- Never commit actual credentials to version control
- Use a separate `secrets.hpp` file for production deployments
- Consider adding `secrets.hpp` to your `.gitignore` file

### 2. MQTT Broker Configuration

The firmware is configured to connect to an MQTT broker at `192.168.1.200:1883`. If you need to change this:

1. Open `src/main.cpp`
2. Modify the MQTT server configuration in the `setup()` function:

```cpp
MqttService::instance().setServer(IPAddress(YOUR_BROKER_IP), YOUR_BROKER_PORT);
```

### 2.5. (Optional) Setting Device Id

To set a unique device ID for your drone, you can modify the `TelemetryService` initialization in `src/main.cpp`:

```cpp
TelemetryService::instance().begin("Drone", 100); // telemetry @ 100Hz
```

Replace `"Drone"` with your desired device ID.
This will precede all telemetry data published by this device.
Example: Imu will publish data to the topic `Drone/imu`.

### 3. Build and Upload

1. Make sure you have PlatformIO installed
2. Connect your ESP32 to your computer
3. Build the project:

   ```bash
   pio run
   ```

4. Upload to your ESP32:

   ```bash
   pio run --target upload --upload-port COMX
   ```

   (Replace `COMX` with your ESP32's serial port)

## Project Structure

```text
src/
├── main.cpp                          # Main application entry point
├── secrets.hpp                       # Wi-Fi credentials (configure before use)
├── services/
│   ├── mqtt_service.cpp/.hpp         # MQTT connectivity and messaging
│   └── telemetry_service.cpp/.hpp    # Telemetry data collection
└── telemetry/
    ├── interface_telemetry_provider.hpp  # Telemetry provider interface
    ├── encoders/
    │   └── json_encoder.hpp          # JSON data encoding
    └── sensors/
        └── imu_mpu_9250.cpp/.hpp     # MPU-9250 IMU sensor driver
```

## Dependencies

The project uses the following libraries (automatically managed by PlatformIO):

- `AsyncMqttClient` - Asynchronous MQTT client
- `ArduinoJson` - JSON parsing and generation
- `Adafruit MPU9250` - MPU-9250 sensor library
- `Adafruit Sensor Calibration` - Sensor calibration utilities
- `Adafruit AHRS` - Attitude and Heading Reference System

## Usage

Once uploaded and running, the firmware will:

1. Connect to the configured Wi-Fi network
2. Establish connection to the MQTT broker
3. Begin collecting IMU sensor data at 100Hz
4. Publish telemetry data to MQTT topics under the "Drone (default)" namespace

Monitor the serial output (115200 baud) for connection status and debugging information.

## Troubleshooting

### Wi-Fi Connection Issues

- Verify your Wi-Fi credentials in `secrets.hpp`
- Ensure your ESP32 is within range of the Wi-Fi network
- Check that your network supports 2.4GHz (ESP32 requirement)

### MQTT Connection Issues

- Verify the MQTT broker is running
- Ensure the MQTT broker is not running localhost only
- Check the IP address and port configuration
- Ensure firewall settings allow MQTT traffic

### Build Issues

- Make sure all dependencies are properly installed via PlatformIO
- Check that your ESP32 board is properly selected in `platformio.ini`

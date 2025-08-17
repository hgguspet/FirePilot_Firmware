# FirePilot MQTT API

This document describes how FirePilot Firmware interacts with MQTT.

The goal is to keep MQTT communication **predictable**, **versioned**, and **easy to integrate** with other systems.

---

## Overview

- **Root namespace:** `<deviceId>`
- **Topic format:** `<deviceId>/<category>/<subtopic>`
- **Encoding:** UTF-8 JSON unless otherwise noted
- **QoS:** `0` by default (may vary per topic)
- **Retain:** `false` by default (may vary per topic)
- **Timestamps (may or may not be provided):**  
  - Field `t` → integer, **microseconds since boot**  
  - If wall clock time is available, an additional `time` field (ISO-8601 UTC) may be included.

---

## Device ID

`<deviceId>` can uniquely identify each flight controller.

- **Default:** `"Drone"`
- **Configurable:** Can be changed in `main.cpp` before deployment.

---

## Publishing Topics

| Category    | Description                                  |
|-------------|----------------------------------------------|
| `log`       | Log messages at various severity levels      |
| `telemetry` | Sensor streams (IMU, GPS, battery, etc.)     |

---

## Subscribed Topics

| Category | Description |
|----------|-------------|
| *(none)* | Currently, no controller commands are implemented via MQTT subscriptions. |

---

*Note:* Future versions may add subscription topics for remote commands, configuration updates, and OTA triggers.

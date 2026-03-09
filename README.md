# ESP32 WROVER – 4 Pots + MQTT + Camera Image

Firmware for ESP32 WROVER: 4 pots (4x soil moisture), BH1750 light sensor (I2C), water level, DHT11 temperature/humidity, relay with web toggle. WiFi setup via AP + HTML form; sensor data published to HiveMQ every 1 minute; camera image included in data payload (when enabled).

## Sensors & pins

| Sensor             | GPIO / Bus     |Notes                              |
|--------------------|----------------|-----------------------------------|
| Soil moisture 1    | GPIO 1         |POT1                               |
| Soil moisture 2    | GPIO 3         |POT2                               |
| Soil moisture 3    | GPIO 33        |POT3                               |
| Soil moisture 4    | GPIO 32        |POT4                               |
| BH1750 light       | I2C SDA=21, SCL=22 | Address 0x23 (ADDR pin to GND) |
| Water level        | GPIO 39        | Analog level sensor               |
| DHT11 (temp/hum)   | GPIO 4         | DHT11 Module                      |
| Relay              | GPIO 5         | Output (pump / valve)             |

## BH1750 wiring

| BH1750 Pin | ESP32 Pin |
|------------|-----------|
| VCC        | 3.3V      |
| GND        | GND       |
| SDA        | GPIO 21   |
| SCL        | GPIO 22   |
| ADDR       | GND       |

## Libraries (Arduino Library Manager)

- **PubSubClient** (Nick O'Leary)
- **ArduinoJson** (Benoit Blanchon)
- **DHTesp** (beegee-tokyo) – for DHT11
- **BH1750** (Christopher Laws) – digital light sensor
- **esp32-camera** – included with the ESP32 board package. Enable with `#define ENABLE_CAMERA 1`.

## MQTT (HiveMQ)

MQTT broker details are **hardcoded** in the sketch (not configurable from the web page):

```c
#define HIVEMQ_BROKER  "broker.hivemq.com"
#define HIVEMQ_PORT    1883
#define HIVEMQ_USER    ""
#define HIVEMQ_PASS    ""
```

To use a private HiveMQ Cloud cluster, edit these four `#define` values and re-upload.

## First run / WiFi setup

1. Flash the sketch. If no WiFi is configured (or connection fails), the ESP32 starts AP **ESP32-Plot-Setup** (password: `12345678`).
2. Connect a phone/PC to that AP and open **http://192.168.4.1**.
3. Fill the form:
   - **Plot ID** – set once; after save it becomes read-only.
   - **WiFi SSID** – scan for networks or type manually.
   - **WiFi Password**.
4. Click **Save & Connect**. The ESP32 restarts, connects to WiFi, and begins publishing.

To change config later: open the setup page and use **Reset configuration**.

## Web pages (on WiFi)

Once connected to WiFi, visit `http://<ESP32_IP>/`:

| Path          | Description                                      |
|---------------|--------------------------------------------------|
| `/`           | Setup page (Plot ID, WiFi)                       |
| `/dashboard`  | Live sensor data + camera image + relay toggle   |
| `/api/plot`   | JSON API – current sensor data + cached image    |
| `/api/relay`  | GET = relay state; POST `?state=on` or `?state=off` |
| `/cam`        | Test camera – take a snapshot                    |
| `/snapshot`   | Raw JPEG snapshot                                |

## Relay control

The relay can be toggled from the **dashboard** using the on-screen switch, or via the API:

```
POST http://<ESP32_IP>/api/relay?state=on
POST http://<ESP32_IP>/api/relay?state=off
GET  http://<ESP32_IP>/api/relay        → {"relay":"on"} or {"relay":"off"}
```

## MQTT data (every 1 minute)

**Topic:** `plot/<PLOT_ID>/data`

```json
{
  "id": "plot123",
  "ip": "192.168.1.50",
  "pots": [
    { "name": "POT1", "moisture": 45, "temperature": 25.2, "humidity": 60.5, "waterLevel": 60, "lightIntensity": 340 },
    { "name": "POT2", "moisture": 52, "temperature": 25.2, "humidity": 60.5, "waterLevel": 60, "lightIntensity": 340 },
    { "name": "POT3", "moisture": 38, "temperature": 25.2, "humidity": 60.5, "waterLevel": 60, "lightIntensity": 340 },
    { "name": "POT4", "moisture": 61, "temperature": 25.2, "humidity": 60.5, "waterLevel": 60, "lightIntensity": 340 }
  ],
  "image": "<base64 JPEG>"
}
```

- **moisture**: per-pot soil moisture (0–100%).
- **temperature / humidity**: shared from DHT11.
- **waterLevel**: shared analog sensor (0–100%).
- **lightIntensity**: shared from BH1750, in **lux** (0 = dark, ~500 = office, 10000+ = sunlight).
- **image**: base64-encoded JPEG from camera (included when camera is enabled and working).

## Camera image (optional)

With `#define ENABLE_CAMERA 1`:

- A cached JPEG image is included as base64 in the `/data` MQTT payload every 1 minute.
- A separate raw JPEG is also published to `plot/<PLOT_ID>/image` at the configured image interval.
- Default camera config supports **ESP32 WROVER KIT** (`CAMERA_BOARD_WROVER_CAM 1`) and **AI-Thinker ESP32-CAM** (`CAMERA_BOARD_WROVER_CAM 0`).
- **PSRAM required** – in Arduino IDE set **Tools > PSRAM > OPI PSRAM** (or Enabled).
- To disable camera entirely, set `#define ENABLE_CAMERA 0`.

# Drone Attitude Monitor
**ESP32 + MPU6050 + MAVLink v2 + QGroundControl + GPIO Web Panel**

A real-time 6-DOF attitude telemetry system I built from scratch on an ESP32. It reads raw IMU data from an MPU6050, runs a Mahony AHRS filter to compute stable roll/pitch/yaw angles, and streams them wirelessly to QGroundControl via MAVLink v2 over WiFi UDP — the same protocol used in real drone autopilots like ArduPilot and PX4.

The ESP32 also hosts a browser-based GPIO control panel accessible at `192.168.4.1`, letting you toggle digital outputs, control servos via PWM, and read ADC voltages — all from any browser on the same WiFi network.

> Built as a major portfolio project during my BS Electronic Systems program at IIT Madras.

---

## Demo

<!-- Add demo GIF here — board tilting, QGC HUD responding -->

**QGC connected and receiving live attitude data:**

<!-- Add QGC screenshot here -->

**Hardware setup:**

<!-- Add wiring photo here -->

---

## What it does

The system has three parallel interfaces running simultaneously:

| Interface | Protocol | What you see |
|-----------|----------|-------------|
| QGroundControl | MAVLink v2 over UDP | Artificial horizon, roll/pitch/yaw HUD |
| Browser dashboard | WebSocket JSON | Live angle readouts, GPIO state |
| Browser GPIO panel | HTTP REST API | Toggle pins, control servo, read ADC |

ESP32 runs as a WiFi Access Point at `192.168.4.1`. No router needed — connect directly.

---

## System architecture

```
MPU6050
  │
  │ I2C (SDA=GPIO21, SCL=GPIO22)
  ▼
ESP32
  ├── Mahony AHRS filter (100Hz)
  │
  ├── MAVLink v2 encoder
  │     └── UDP port 14550 ──► QGroundControl (HUD)
  │
  ├── WebSocket server
  │     └── ws://192.168.4.1/ws ──► Browser (live data)
  │
  └── HTTP REST API
        └── http://192.168.4.1 ──► Browser GPIO panel
```

---

## Hardware

| Component | Purpose |
|-----------|---------|
| ESP32 DevKit v1 | Main MCU — WiFi, sensor processing, GPIO |
| MPU6050 | 6-axis IMU: 3-axis accelerometer + 3-axis gyroscope |
| LED (GPIO 2) | Built-in digital output demo |
| Servo motor (GPIO 13) | PWM output — 50Hz, 0–180° control |
| Push button (GPIO 5) | Digital input with internal pullup |
| Potentiometer (GPIO 34) | ADC input — reads 0–3.3V |

---

## Wiring

| MPU6050 pin | ESP32 pin | Notes |
|-------------|-----------|-------|
| VCC | 3.3V | ⚠️ Do NOT use 5V |
| GND | GND | Common ground |
| SDA | GPIO 21 | I2C data |
| SCL | GPIO 22 | I2C clock |
| AD0 | Unconnected | Floats low → I2C address 0x68 |

---

## Project structure

Each folder is a self-contained Arduino sketch representing one stage of the build. You can open and run any stage independently.

```
drone-attitude-monitor/
├── 01_i2c_scanner/         # Confirms MPU6050 alive at 0x68
├── 02_raw_reads/           # Reads raw accel + gyro registers directly
├── 03_calibration/         # Removes sensor offsets, corrects to zero
├── 04_mahony_filter/       # Sensor fusion → stable roll/pitch/yaw
├── 05_mavlink_qgc/         # MAVLink v2 over WiFi UDP → QGC HUD
├── 06_full_system/         # Everything: MAVLink + WebSocket + GPIO API
├── docs/
│   ├── wiring.md           # Detailed wiring guide
│   ├── serial_output/      # Screenshots of each stage output
│   └── qgc_screenshots/    # QGC connected and HUD screenshots
└── README.md
```

---

## How it works — the interesting parts

### Why Mahony filter?

Raw gyroscope data drifts over time — integrate it long enough and angles become meaningless. Raw accelerometer data is too noisy for fast movement. The Mahony filter solves both: the gyro handles fast rotations accurately, and the accelerometer continuously corrects long-term drift. The internal state is a quaternion (4 numbers representing 3D orientation) which avoids gimbal lock — a problem that Euler angles alone can't handle.

### Why MAVLink?

MAVLink is the industry standard protocol for drone telemetry — used by ArduPilot, PX4, and virtually every commercial autopilot. It's a binary protocol with built-in CRC, sequence numbers, and system/component IDs. Using it means QGroundControl — a professional GCS used on real drones — understands your ESP32 out of the box. No custom parsing, no proprietary format.

### Why UDP over TCP?

Attitude data is time-sensitive. If a packet is lost, you don't want to wait for retransmission — you want the next fresh reading immediately. UDP is fire-and-forget: lower latency, no handshaking overhead, ideal for real-time sensor streaming. QGC uses UDP by default for exactly this reason.

---

## GPIO HTTP API

The ESP32 serves a REST API for direct GPIO control from any HTTP client (browser, curl, Postman):

| Endpoint | Action | Example |
|----------|--------|---------|
| `GET /gpio/set?pin=2&val=1` | Set digital pin HIGH | Turn LED on |
| `GET /gpio/set?pin=2&val=0` | Set digital pin LOW | Turn LED off |
| `GET /gpio/pwm?pin=13&duty=90` | Set servo angle | Move servo to 90° |
| `GET /gpio/read?pin=5` | Read digital pin | Returns 0 or 1 |
| `GET /adc?pin=34` | Read ADC voltage | Returns voltage in V |

---

## WebSocket stream

Connect to `ws://192.168.4.1/ws` to receive live JSON every 50ms:

```json
{
  "roll": 12.4,
  "pitch": -3.1,
  "yaw": 87.2,
  "pins": { "2": 1, "5": 0 },
  "adc": { "34": 1.65 }
}
```

---

## Setup and usage

### Requirements
- Arduino IDE 2.x with ESP32 board support
- Libraries: AsyncTCP, ESP Async WebServer (via Library Manager)
- MAVLink c_library_v2 (manual install — see below)
- QGroundControl (free download at qgroundcontrol.com)

### Install MAVLink library
```bash
cd ~/Documents/Arduino/libraries
git clone https://github.com/mavlink/c_library_v2.git mavlink
```

### Flash and run
1. Open `06_full_system/06_full_system.ino` in Arduino IDE
2. Select board: ESP32 Dev Module, port: your COM port
3. Upload
4. Place board flat on desk — calibration runs automatically (2 seconds)
5. Connect PC WiFi to **DroneMonitor** (password: `12345678`)
6. Open QGC → Comm Links → Add → UDP → port 14550 → Connect
7. Open browser → `http://192.168.4.1`

---

## What I learned building this

Working through this project layer by layer — starting from raw I2C register reads all the way up to wireless MAVLink telemetry — gave me a solid understanding of how real embedded telemetry systems are structured. The most interesting part was the sensor fusion: seeing why gyro-only or accel-only approaches fail, and understanding why the Mahony filter works so well for attitude estimation at 100Hz on a microcontroller with limited floating point performance.

The MAVLink integration was also eye-opening — understanding binary protocol framing, CRC calculation, and how a GCS discovers and handshakes with a vehicle taught me a lot about how professional drone systems communicate.

---

## Skills demonstrated

- MAVLink v2 protocol implementation from register level up
- AHRS sensor fusion (Mahony complementary filter) at 100Hz
- I2C peripheral interfacing and register-level driver development
- Wireless UDP telemetry via ESP32 SoftAP
- REST HTTP API design on embedded hardware
- WebSocket real-time bidirectional streaming
- QGroundControl GCS integration
- Full GPIO control: digital output, PWM/servo, ADC, digital input
- Embedded C++ on ESP32

---

## License

MIT License — feel free to use, modify, and build on this.
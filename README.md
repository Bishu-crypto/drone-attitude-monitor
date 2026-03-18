# drone-attitude-monitor

# ESP32 + MPU6050 + MAVLink v2 + QGroundControl + GPIO Web Panel

Real-time 6-DOF attitude telemetry system. Streams orientation data to QGroundControl via MAVLink v2 over WiFi UDP, and to a browser-based GPIO control panel via WebSocket + HTTP API.

Repo:https://github.com/Bishu-crypto/drone-attitude-monitor



## System Architecture

MPU6050 ──I2C──► ESP32 ──MAVLink/UDP──► QGroundControl (HUD)
                   ├──WebSocket──► Browser (live roll/pitch/yaw)
                   └──HTTP API──► Browser GPIO panel




## Hardware
| Component | Purpose |
|-----------|---------|
| ESP32 DevKit v1 | Main MCU — WiFi, processing, GPIO |
| MPU6050 | 6-axis IMU: accel + gyro |
| LED / Servo / Button / Pot | GPIO demo peripherals |

## Wiring — MPU6050 to ESP32
| MPU6050 | ESP32 | Notes |
|---------|-------|-------|
| VCC | 3.3V | NOT 5V |
| GND | GND | |
| SDA | GPIO 21 | |
| SCL | GPIO 22 | |



## Project Structure

01_i2c_scanner - Confirms MPU6050 at address 0x68  
02_raw_reads - Reads raw accel + gyro registers 
03_calibration - Removes sensor offsets, corrects to 0 
04_mahony_filter - Sensor fusion → roll/pitch/yaw angles 
05_mavlink_qgc - MAVLink over WiFi UDP → QGC HUD 
06_full_system - GPIO web panel + WebSocket + MAVLink 



## GPIO HTTP API
| Endpoint | Action |
|----------|--------|
| GET /gpio/set?pin=2&val=1 | Set pin HIGH |
| GET /gpio/set?pin=2&val=0 | Set pin LOW |
| GET /gpio/pwm?pin=13&duty=90 | Servo to 90° |
| GET /gpio/read?pin=5 | Read digital pin |
| GET /adc?pin=34 | Read ADC voltage |


## How to use
1. Flash `06_full_system` firmware to ESP32
2. Connect PC WiFi to DroneMonitor
3. Open QGC → Comm Links → UDP → port 14550 → Connect




## Skills demonstrated
- MAVLink v2 protocol implementation from scratch
- AHRS sensor fusion (Mahony filter) at 100Hz
- Wireless UDP telemetry via ESP32 SoftAP
- REST HTTP API on embedded hardware
- WebSocket real-time streaming
- QGroundControl GCS integration
- Full GPIO control: digital, PWM/servo, ADC, digital input



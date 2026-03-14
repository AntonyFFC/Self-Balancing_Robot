# Self-Balancing Robot

A two-wheel self-balancing robot project combining embedded systems firmware, control theory, a cross-platform mobile app, and data-visualisation tools.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Repository Structure](#repository-structure)
- [Firmware Variants](#firmware-variants)
  - [STM32 (primary)](#stm32-primary)
  - [ESP32 (WiFi)](#esp32-wifi)
  - [Arduino IDE](#arduino-ide)
- [Control Algorithm](#control-algorithm)
- [Mobile App](#mobile-app)
- [Visualisation Tools](#visualisation-tools)
- [Simulations](#simulations)
- [CAD Models](#cad-models)
- [Getting Started](#getting-started)
- [License](#license)

---

## Overview

The robot maintains vertical balance on two wheels using a real-time PID control loop running at 100 Hz. An MPU6050 IMU provides pitch-angle feedback via a complementary filter. The controller outputs PWM signals to an L298N dual H-bridge motor driver. Wireless communication is handled by either an nRF24L01+ radio module (STM32 variant) or built-in WiFi (ESP32 variant).

---

## Hardware

| Component | Details |
|---|---|
| **Microcontroller** | STM32F103 (primary) · ESP32 (alternative) |
| **IMU** | MPU6050 – 6-axis accelerometer & gyroscope (I²C) |
| **Motor Driver** | L298N dual-channel H-bridge |
| **Wireless** | nRF24L01+ 2.4 GHz RF module (STM32) · Built-in WiFi (ESP32) |
| **Motors** | DC geared motors with PWM speed control |

### STM32 Pinout

![STM32 Nucleo F103RB pinout](Pictures/stm32nucleoF103RB_pinout.png)

### Chassis

![Chassis model](Pictures/chasis_model.png)

---

## Repository Structure

```
SelfBalancing-Robot/
├── stm32_s-b_r/                   # STM32F1 firmware (primary, CubeIDE)
├── stm32_s-b_r_t/                 # STM32F1 alternative / test build
├── esp32_s-b_r/                   # ESP32 firmware (ESP-IDF / CMake)
├── esp32_s-b_r_fromArIDE/         # ESP32 firmware (Arduino IDE style)
├── self-balancing-robot_arduinoIDE/# Original Arduino IDE version
├── robot_control_app/             # Flutter cross-platform mobile app
├── Python_Plotting/               # Real-time UDP data visualisation
├── csv_py_plot/                   # CSV playback plotting
├── simulations/                   # MATLAB/Simulink models & analysis
├── CAD/                           # FreeCAD & STL 3D models
├── Pictures/                      # Project images
├── diagrams/                      # Architecture diagrams (Visual Paradigm)
└── Self-Balancing Robot.ssproj    # Serial Studio monitoring project
```

---

## Firmware Variants

### STM32 (primary)

Located in `stm32_s-b_r/`. Built with STM32CubeIDE.

- **Control loop:** TIM3 interrupt at 100 Hz (10 ms period)
- **Sensor:** MPU6050 via I²C
- **Wireless:** nRF24L01+ via SPI – 4-byte payload (servo + motor ADC values)
- **Motor output:** PWM via TIM1 channels, with deadband compensation (22% minimum duty cycle)

**Build:** Open `stm32_s-b_r/` as an STM32CubeIDE project and build with the standard toolchain.

---

### ESP32 (WiFi)

Located in `esp32_s-b_r/` (ESP-IDF) and `esp32_s-b_r_fromArIDE/` (Arduino IDE style).

- **Sensor:** MPU6050/MPU6500 via I²C
- **Telemetry TX:** UDP port **7777** – `PITCH,SETPITCH,CONTROL_SIGNAL\n`
- **Command RX:** UDP port **7778** – live PID parameter updates (`P=2.5,I=0.001,D=0.0\n`)
- **FreeRTOS** task architecture

**Build (ESP-IDF):**
```bash
cd esp32_s-b_r
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

---

### Arduino IDE

Located in `self-balancing-robot_arduinoIDE/`. Open the `.ino` sketch in the Arduino IDE and flash to a compatible board.

---

## Control Algorithm

### Pitch Estimation – Complementary Filter

```
pitch = 0.95 × (pitch + gx × dt) + 0.05 × acc_pitch
```

- 95% weight on gyroscope integration (high-frequency accuracy)
- 5% weight on accelerometer angle (long-term drift correction)
- Accelerometer pitch: `atan2(ax, sqrt(ay² + az²))`

### Discrete PID Controller

```
K  = 2.5       (proportional gain)
Ti = 900 000   (integral time constant)
Td = 0.0       (derivative time constant – disabled)
Tp = 0.01 s    (sampling period, 10 ms)
```

Motor PWM output is clamped and a deadband of 22% is applied to overcome static friction.

---

## Mobile App

Located in `robot_control_app/`. Built with Flutter (SDK ≥ 3.7.2). Targets Android, iOS, Windows, Linux, macOS, and Web.

**Features:**
- Real-time pitch, set-pitch, and control-signal charts (custom `CustomPainter`)
- PID parameter sliders with live UDP updates to the robot
- Manual speed/direction control
- Data logging to CSV with export via `share_plus`

**Screenshots:**

| PID Tuning | Manual Control | Limits | Connection Status |
|---|---|---|---|
| ![PID](Pictures/app_PID.jpg) | ![Manual](Pictures/app_manual.jpg) | ![Limits](Pictures/limits_app.jpg) | ![Connection](Pictures/connection%20status.jpg) |

**Build & run:**
```bash
cd robot_control_app
flutter pub get
flutter run
```

---

## Visualisation Tools

### Python Real-Time Plotter

Located in `Python_Plotting/`. Receives UDP telemetry from the ESP32 and plots pitch, setpoint, and control signal in real time using `matplotlib`.

```bash
cd Python_Plotting
pip install matplotlib numpy
python dashboard.py
```

![Python Plotter](Pictures/PythonPlotter1.png)
![Python Plotter plots](Pictures/PythonPlotter2.png)

### Serial Studio

A pre-built Serial Studio project (`Self-Balancing Robot.ssproj`) parses the comma-separated telemetry stream and displays three datasets: Pitch, Set Pitch, and Control Signal.

![Serial Studio](Pictures/Serial_Studio_PScr.png)
![Serial Studio plots](Pictures/Serial_Studio_PScr_plots.png)

### CSV Plotter

Located in `csv_py_plot/`. Replays recorded CSV data for offline analysis.

---

## Simulations

Located in `simulations/`. Requires MATLAB with Simulink.

| File | Description |
|---|---|
| `model.slx` | Continuous-time Simulink model |
| `Dyskretny_PID.mlx` | Discrete PID controller design |
| `transmitancja.mlx` | Transfer function analysis |
| `zigler_nichols.mlx` | Ziegler–Nichols tuning method |
| `Simple_G/robot_imitation.m` | Simplified robot dynamics simulation |

---

## CAD Models

Located in `CAD/`. Designed in FreeCAD.

| File | Description |
|---|---|
| `Chassy1.FCStd` | Full parametric chassis model |
| `Chassy1.stl` | Exportable mesh for 3D printing |

---

## Getting Started

1. **Clone the repository**
   ```bash
   git clone https://github.com/AntonyFFC/SelfBalancing-Robot.git
   cd SelfBalancing-Robot
   ```

2. **Choose a firmware variant** – see [Firmware Variants](#firmware-variants) above.

3. **Flash the firmware** to your microcontroller.

4. **Run the mobile app or Python plotter** to monitor and tune the robot wirelessly.

5. *(Optional)* Open the MATLAB simulations to analyse the control system or experiment with different PID parameters.

---

## License

This project is provided as-is for educational purposes. See the repository for any applicable licence information.

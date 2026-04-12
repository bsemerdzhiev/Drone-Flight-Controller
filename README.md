# Drone Flight Controller

A full-stack quadcopter flight controller built from scratch in **Rust** on the **nRF51822** (ARM Cortex-M0), paired with a **Python** ground station featuring real-time telemetry, live 3D visualization, and BLE wireless communication via Nordic UART Service (NUS).

This project was developed as part of the **Embedded Systems Lab** at TU Delft (CESE4030), using the `tudelft_quadrupel` hardware platform.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Drone (nRF51822)                     │
│                                                         │
│  ┌─────────────┐   ┌──────────────┐   ┌──────────────┐  │
│  │  Sensor     │   │  State       │   │  Control     │  │
│  │  Fusion     │──▶│  Estimation  │──▶│  Loop        │  │
│  │  (Compl.    │   │  (Quat/Euler │   │              │  │ 
│  │   Filter)   │   │   + Kalman)  │   │  (PID)       │  │
│  └─────────────┘   └──────────────┘   └──────────────┘  │
│         │                                     │         │
│    MPU-6050 IMU                        Motor Mixing     │
│    MS5611 Baro                      (Thrust/Drag Model) │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │         BLE / NUS (Nordic S110 SoftDevice)       │   │
│  │         HDLC-framed telemetry packets            │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────┬───────────────────────────────┘
                          │ BLE/UART
┌─────────────────────────▼────────────────────────────────┐
│                  Host PC (Linux)                         │
│                                                          │
│  ┌──────────────┐   Unix Socket   ┌──────────────────┐   │
│  │  runner      │◀───────────────▶│  Python UI       │   │
│  │  (Rust/x86)  │                 │  (DearPyGUI      │   │
│  │  BLE bridge  │                 │   + pyqtgraph    │   │
│  │ +UART+flasher│                 │   3D OpenGL)     │   │
│  └──────────────┘                 └──────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Drone-Flight-Controller/
├── dronecode/          # Firmware — runs on the nRF51822 (thumbv6m-none-eabi)
├── runner/             # Host-side tool — flashes firmware, bridges BLE ↔ Unix socket
├── my-hdlc/            # Custom HDLC framing library (shared crate)
├── ble_setup/          # BLE configuration / Nordic NUS C bindings
├── tudelft-quadrupel/  # TU Delft HAL library (git submodule)
├── ui/                 # Python ground station (DearPyGUI + pyqtgraph)
├── memory.x            # Linker script for nRF51822 flash/RAM layout
├── rust-toolchain.toml # Pinned Rust toolchain (thumbv6m target)
├── Cargo.toml          # Cargo workspace root
├── requirements.txt    # Python UI dependencies
└── pid_values.json     # Saved PID tuning presets
```

---

## Firmware (`dronecode`)

The flight controller firmware runs bare-metal on the **nRF51822 (ARM Cortex-M0)** using the `tudelft_quadrupel` HAL. All arithmetic is **fixed-point** throughout — no floating-point hardware on Cortex-M0.

### Sensor Fusion & State Estimation

- **Complementary filter** — fuses MPU-6050 gyroscope and accelerometer data to produce roll/pitch/yaw attitude estimates as quaternions, converted to Euler angles for control
- **Kalman filter** — applied to MS5611 barometric pressure readings for altitude estimation and height control
- Careful handling of gyroscope LSB scaling and quaternion-to-Euler edge cases (overflow, sign conventions)

### Control

- **PID** architecture — Regular PID controller that compares angles and rates to calculate motor correction values
- **Height controller** — altitude hold using barometer-derived altitude set-point
- **Motor mixing** — rotor commands derived from a thrust/drag coefficient model mapping PID outputs to individual motor PWM values

### Communication

- Telemetry is **HDLC-framed** for reliable packetized transmission over the wireless link
- Packets are serialized using [`postcard`](https://crates.io/crates/postcard) (compact `no_std` binary format)
- BLE transport uses the **Nordic S110 SoftDevice** with the **Nordic UART Service (NUS)**
- The NUS C library is linked into Rust via a `cc`-crate FFI bridge (`ble-c-extern` crate)

### BLE Setup

THe S110 SoftDevice can come pre-flashed with the board, or be additionally flashed via **SWD** using a Nucleo board's ST-Link with pogo pins. The soft device occupies the low flash region; the application image is placed above it per the linker script.

---

## Host Runner (`runner`)

A native x86 Rust binary that:

1. Flashes the compiled `dronecode` ELF to the drone over UART.
2. Opens a BLE connection to the drone's NUS service.
3. Bridges BLE NUS/UART ↔ a Unix domain socket, making telemetry available to the Python UI.

---

## Ground Station (`ui`)

A Python application providing real-time monitoring and control.

**Features:**
- **DearPyGUI** dashboard with tabbed layout — live telemetry vs. logged session data
- **pyqtgraph OpenGL** 3D visualization — renders drone orientation in real time from quaternion telemetry
- Live PID tuning — send updated gain values to the drone mid-flight
- Unix socket IPC to `runner` for telemetry ingestion
- Coordinate frame alignment between sensor frame, body frame, and visualization frame

**Run the UI:**
```bash
pip install -r requirements.txt
cd ui
python main.py
```

---

## Build & Flash

### Prerequisites

- Rust with the `thumbv6m-none-eabi` target installed.
- `arm-none-eabi-gdb` and `OpenOCD` for SWD debugging.
- Nordic S110 SoftDevice flashed to the drone (one-time setup).

### Build firmware

```bash
# From workspace root
cargo build --release
```

### Flash & run

```bash
cargo run --release
```

This compiles the firmware, uploads it via the runner, and starts the BLE bridge.

---

## Cargo Workspace Crates

| Crate | Target | Purpose |
|---|---|---|
| `dronecode` | `thumbv6m-none-eabi` | Flight controller firmware |
| `runner` | `x86_64-unknown-linux-gnu` | Host flasher + BLE bridge |
| `my-hdlc` | both | HDLC packet framing/deframing |
| `ble-c-extern` | `thumbv6m-none-eabi` | FFI wrapper for Nordic NUS C library |

---

## PID Tuning

Saved gain presets are stored in `pid_values.json`. The ground station UI allows live gain updates to be sent to the drone without re-flashing.

Relevant axes and loops:

| Axis | (Values) |
|---|---|
| Roll | Kp, Ki, Kd |
| Pitch | Kp, Ki, Kd |
| Yaw | Kp, —, — |
| Height | Kp, Ki, Kd |

---

## Hardware

| Component | Part |
|---|---|
| MCU | nRF51822 (ARM Cortex-M0, 256 KB flash, 16 KB RAM) |
| IMU | MPU-6050 (gyroscope + accelerometer) |
| Barometer | MS5611 |
| BLE Stack | Nordic S110 SoftDevice v8 |
| Debug/Flash | ST-Link via Nucleo + pogo-pin SWD |

---

## License

Academic project — TU Delft Embedded Systems Lab (CESE4030), 2025–2026.

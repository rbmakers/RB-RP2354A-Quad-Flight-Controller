# RB-RP2354A Quadcopter Flight Controller

> **A complete, Betaflight-class open-source flight stack built on the Raspberry Pi RP2354A.**  
> Supports 65 mm coreless TinyWhoops and 2″–5″ BLDC racing quads from the same hardware.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Dual-Core Execution Model](#2-dual-core-execution-model)
3. [Attitude & Heading Reference System (AHRS)](#3-attitude--heading-reference-system-ahrs)
4. [Dual PID Control Loop](#4-dual-pid-control-loop)
5. [Switchable Motor Output — Coreless vs. BLDC](#5-switchable-motor-output--coreless-vs-bldc)
6. [DSP and FPU — Accelerating Complex Mathematics](#6-dsp-and-fpu--accelerating-complex-mathematics)
7. [Hardware FIFO — Keeping the Control Loop Uninterrupted](#7-hardware-fifo--keeping-the-control-loop-uninterrupted)
8. [Programmable I/O (PIO) — Precision Motor Signalling](#8-programmable-io-pio--precision-motor-signalling)
9. [Gyro Filter Pipeline](#9-gyro-filter-pipeline)
10. [Telemetry, Blackbox, and Safety Systems](#10-telemetry-blackbox-and-safety-systems)
11. [Performance Summary](#11-performance-summary)
12. [Quick-Start Guide](#12-quick-start-guide)

---

## 1. Overview

The **RB-RP2354A** is an education-focused quadcopter flight controller built around the Raspberry Pi **RP2354A** microcontroller.  It is designed to expose students and researchers to the full depth of a modern flight stack — from bare-metal hardware drivers through signal processing, sensor fusion, and closed-loop control — while remaining fully open-source and approachable in the **Arduino 2.3.x** ecosystem.

| Feature | Specification |
|---|---|
| Microcontroller | RP2354A — Cortex-M33 dual-core, 150 MHz |
| IMU | Bosch BMI088 — separate accel + gyro dice, SPI, up to 2 kHz DRDY |
| Barometer | Bosch BMP580 — I²C, 50 Hz, 24-bit pressure + temperature |
| RC Receiver | ExpressLRS via CRSF — UART, 420 kBaud, 16 channels |
| Brushed motors | 4 × MOSFET PWM (GPIO 29 / 11 / 18 / 25, 32 kHz, 10-bit) |
| BLDC ESC output | 4 × DShot600 PIO (GPIO 1 / 4 / 10 / 14) |
| Blackbox storage | W25Q128 SPI flash, 16 MiB *(optional — not populated on current board revision)* |
| Battery sense | 12-bit ADC — voltage divider on GPIO 27 *(current sense not implemented)* |
| Build environment | Arduino IDE 2.3.8 + earlephilhower/arduino-pico ≥ 3.x |

---

## 2. Dual-Core Execution Model

The RP2354A integrates **two independent Cortex-M33 processors** that share system memory but run completely separate instruction streams simultaneously.  The firmware partitions work into a hard real-time domain on **Core 0** and an I/O-intensive domain on **Core 1**, so that no peripheral activity — UART parsing, telemetry formatting, FFT computation — can ever delay the critical attitude loop.

```
┌─────────────────────────────────┐   ┌─────────────────────────────────┐
│         CORE 0  (150 MHz)       │   │         CORE 1  (150 MHz)       │
│   Hard real-time flight loop    │   │   I/O & analysis                │
├─────────────────────────────────┤   ├─────────────────────────────────┤
│ BMI088 DRDY ISR  @ 2 kHz       │   │ ELRS / CRSF parser  (async)    │
│  → RPM notch filter bank       │   │ CMSIS-DSP FFT  @ ~8 Hz         │
│  → Biquad LPF  (3 axes)        │   │  → update dynamic notch freq   │
│  → Madgwick AHRS               │   │                                 │
│ 1 kHz control loop:            │   │  Shared via lock-free           │
│  → Altitude estimation         │   │  volatile atomic copy           │
│  → Flight-mode logic           │   │                                 │
│  → Dual PID  (rate + angle)    │   │                                 │
│  → Quad-X motor mixer          │   │                                 │
│  → DShot600 PIO / brushed PWM  │   │                                 │
│  → Blackbox write  *(optional)*  │   │                                 │
│  → USB telemetry @ 100 Hz      │   │                                 │
└─────────────────────────────────┘   └─────────────────────────────────┘
              ↑ shared SRAM  (atomic snapshot on read) ↑
```

### 2.1 Core 0 — Hard Real-Time Domain

Core 0 owns every time-critical operation.  The BMI088 gyroscope asserts a Data-Ready (DRDY) interrupt at **exactly 2 000 Hz**; Core 0 services this ISR, runs the full gyro filter chain, and updates the Madgwick quaternion.  A polled **1 kHz loop** then reads the latest AHRS output, computes PID corrections, mixes them to motor commands, and writes those commands to either the brushed PWM peripherals or the PIO DShot state machines.

The entire control path — sensor to actuator — completes within **≈ 50 µs**, leaving Core 0 with over 85 % idle time even with all optional features enabled.

### 2.2 Core 1 — I/O and Analysis Domain

Core 1 runs an infinite loop that continuously drains the CRSF UART receive FIFO, decoding ExpressLRS packets and updating the shared RC channel table.  Optionally it feeds gyro samples into a 256-point CMSIS-DSP real-FFT, detects the dominant vibration frequency, and pushes an updated notch-filter centre frequency back to Core 0 via a volatile atomic variable.

Because Core 1 **never touches motor outputs or PID state**, any delay on Core 1 is entirely invisible to the control loop.

> **Key isolation benefit:** Serial UART at 420 kBaud produces an interrupt every 24 µs at peak data rate.  By confining CRSF parsing to Core 1, these interrupts are invisible to the 2 kHz gyro ISR on Core 0 — eliminating a major source of jitter that single-core flight controllers must manage with interrupt priority schemes.

---

## 3. Attitude & Heading Reference System (AHRS)

The flight controller implements **Sebastian Madgwick's gradient-descent orientation filter** (2010) to fuse BMI088 gyroscope and accelerometer data into a unit quaternion representing the full 3-D orientation of the airframe in real time.

### 3.1 Why a Quaternion Representation?

Euler angles (roll, pitch, yaw) suffer from **gimbal lock** when two rotation axes align.  A unit quaternion **q = [q₀, q₁, q₂, q₃]** represents any orientation without singularities, requires only four floating-point values, and composes orientations with a single multiplication — far cheaper than chaining 3×3 rotation matrices.

### 3.2 Madgwick Filter Algorithm

At each sensor sample the filter:
1. **Propagates** the quaternion forward using gyroscope angular rates via numerical integration of the quaternion kinematics equation.
2. **Corrects** the result with a gradient-descent step that pulls the predicted gravity direction toward the measured accelerometer vector.

The two contributions are weighted by **β** (fusion gain), which trades off gyro-rate responsiveness against accelerometer noise correction.

```
At each IMU sample (2 000 Hz):

┌──────────────┐    ┌──────────────────────────────────────────────────┐
│  BMI088      │    │  Madgwick update step                            │
│  gyro (rad/s)├───►│  1. Rate integration:  q̇ = 0.5 · q ⊗ ωgyro    │
│  accel (g)   ├───►│  2. Gradient descent:                            │
└──────────────┘    │       f  = q ⊗ g_ref ⊗ q* − g_meas             │
                    │       ∇f = J^T · f                               │
                    │       q̇_err = β · ∇f / |∇f|                    │
                    │  3. Combine:   q̇_total = q̇ − q̇_err            │
                    │  4. Integrate: q += q̇_total / fs                │
                    │  5. Normalise: q /= |q|                          │
                    │  6. Extract roll, pitch, yaw  (Euler output)     │
                    └──────────────────────────────────────────────────┘
```

### 3.3 DSP and FPU Acceleration

Each Madgwick update requires approximately **30 single-precision floating-point operations** (multiplications, additions, a square root, normalisation).  The Cortex-M33 FPU makes this feasible at 2 kHz:

| Operation | Cortex-M33 FPU |
|---|---|
| F32 multiply-accumulate | 1 cycle (pipelined) |
| F32 add / subtract | 1 cycle (pipelined) |
| `VSQRT.F32` (hardware) | 14 cycles |
| Complete Madgwick update | ≈ 0.5 µs at 150 MHz |
| CPU load at 2 kHz | **< 0.1 %** |

Without the FPU, a software `sqrt()` on an M0+ core costs ≈ 400 cycles — a **28× penalty** for just one operation.

---

## 4. Dual PID Control Loop

The firmware implements the **two-loop (cascaded) PID structure** used in all modern Betaflight-class controllers.  The outer loop stabilises attitude (angle); the inner loop regulates angular rate.  The two loops run at different speeds and use different sensor signals, giving the aircraft both stable self-levelling and fast disturbance rejection.

```
RC stick input (±1.0)
      │
      ▼
┌─────────────┐  ANGLE MODE:  targetAngle = stick × MaxAngle (45°)
│  Outer Loop │  error = targetAngle − currentAngle  (from Madgwick)
│  (Angle PID)│  output = Kp_outer × error  →  rate setpoint (°/s)
└──────┬──────┘
       │  ACRO MODE:  bypass outer loop — stick maps directly to rate SP
       ▼
┌─────────────┐  error = rateSetpoint − measuredRate  (BMI088 gyro)
│  Inner Loop │  PID = Kp·e  +  Ki·∫e dt  +  Kd·(de/dt)
│  (Rate PID) │  Applied independently to roll, pitch, yaw
└──────┬──────┘
       │
       ▼
┌─────────────┐  Quad-X mixer:
│ Motor Mixer │  M1 (FR) = T + roll + pitch − yaw
│             │  M2 (RR) = T − roll − pitch − yaw
│             │  M3 (RL) = T − roll + pitch + yaw
│             │  M4 (FL) = T + roll − pitch + yaw
└──────┬──────┘
       ▼
  Brushed PWM  or  DShot600 PIO  (compile-time selection)
```

### 4.1 Inner Loop — Angular Rate Control (1 kHz)

| | |
|---|---|
| **Sensor** | BMI088 gyroscope, filtered by biquad LPF + RPM notch bank |
| **Input** | Desired angular rate in rad/s (from outer loop or stick directly in Acro) |
| **Output** | Normalised motor correction, constrained 0–1 after mixing |

The **derivative term** differentiates the measurement rather than the error to avoid "derivative kick" when the setpoint steps.  **Integral anti-windup** clamping prevents motor saturation from causing integrator runaway during aggressive manoeuvres.

### 4.2 Outer Loop — Angle Stabilisation (Angle Mode)

The Madgwick filter outputs roll and pitch angles in degrees.  The outer **P-controller** computes the angular error between the desired tilt (proportional to stick deflection, capped at ±45°) and the measured tilt, then scales it to a rate command fed into the inner loop.

This cascade means the **inner loop always works in angular-rate space** regardless of mode, making PID tuning consistent between Acro and Angle flight.

### 4.3 Altitude Hold — Barometric Sensor Fusion

A vertical-axis PID runs on estimated altitude (metres AGL) derived from a **complementary filter** that fuses BMP580 barometric altitude with Z-axis accelerometer integration:

- **Barometer** → low-frequency absolute altitude (slow, accurate)
- **Accelerometer** → high-frequency velocity updates (fast, drifts)

When altitude hold is engaged via CH7, this PID replaces the raw throttle command.

### 4.4 Ziegler-Nichols Auto-Tune

A **relay-feedback auto-tuner** is included for bench use.  It drives a bang-bang relay oscillation around zero angular rate, measures the ultimate period *Pᵤ* and oscillation amplitude *A*, computes the ultimate gain *Kᵤ = 4d / (π·A)*, then applies the Ziegler-Nichols PID rules:

```
Kp = 0.6 · Ku
Ki = 2 · Kp / Pu
Kd = Kp · Pu / 8
```

The result seeds starting gains for manual fine-tuning on the actual airframe.

---

## 5. Switchable Motor Output — Coreless vs. BLDC

The RB-RP2354A supports two fundamentally different quadcopter classes from the same hardware, with the motor driver selected at **compile time** via a single `#define` in `config.h`.

| Property | Coreless Brushed (`MOTOR_TYPE_BRUSHED`) | BLDC DShot600 (`MOTOR_TYPE_BLDC`) |
|---|---|---|
| GPIO pins | 29, 11, 18, 25 | 1, 4, 10, 14 |
| Drive circuit | Single N-MOSFET per motor | External BLHeli / AM32 ESC |
| Protocol | PWM 32 kHz, 10-bit | DShot600 digital, 600 kbit/s |
| Throttle resolution | 1 024 steps | 2 000 steps (11-bit) |
| ESC calibration | None required | ESC range cal or DShot (auto) |
| RPM telemetry | None | Bidirectional DShot eRPM or BLHeli serial |
| Typical drone class | 65 mm TinyWhoop / micro brushed | 2″–5″ racing or freestyle |

### 5.1 Brushed Motor Path — 32 kHz PWM

Coreless motors are driven by single logic-level N-MOSFETs directly from RP2354A GPIO pins.  PWM frequency is set to **32 kHz** — above human hearing and well above the BMI088 accelerometer passband — so switching noise is invisible to the IMU.  `analogWriteResolution(10)` gives 1 024 throttle steps, which is more than sufficient for micro-quad control.

### 5.2 BLDC Motor Path — DShot600 via PIO

Brushless motors require external ESCs driven by a digital pulse protocol.  DShot600 encodes each throttle command as a **16-bit frame** (11-bit throttle value, 1 telemetry request bit, 4-bit CRC) transmitted at 600 kbit/s.  The firmware implements DShot600 using the RP2354A **PIO hardware** (see [Section 8](#8-programmable-io-pio--precision-motor-signalling)), which delivers all four ESC frames simultaneously with sub-nanosecond jitter and **zero CPU involvement** during transmission.

---

## 6. DSP and FPU — Accelerating Complex Mathematics

The Cortex-M33 core inside the RP2354A includes an **ARMv8-M floating-point unit (FPU)** and optional **DSP instruction-set extensions**.  Both are essential to running a full flight control stack at kilohertz rates on a microcontroller.

### 6.1 FPU — Single-Precision Floating-Point in Hardware

All sensor values and intermediate calculations are IEEE 754 single-precision floats.  Key instruction latencies:

| Operation | Cortex-M33 FPU | Software (M0+) |
|---|---|---|
| `VMLA.F32` multiply-accumulate | 1 cycle | ~5 cycles |
| `VADD.F32` / `VSUB.F32` | 1 cycle | ~5 cycles |
| `VSQRT.F32` | 14 cycles | ~400 cycles |
| `VDIV.F32` | 14 cycles | ~200 cycles |

The Madgwick AHRS update, which performs roughly 60 FP operations, completes in **under 1 µs** at 150 MHz — effectively free in a 1 ms control loop.

### 6.2 DSP Extensions — SIMD Integer Operations

The M33 DSP extension adds SIMD instructions that operate on 32-bit registers as two 16-bit or four 8-bit lanes simultaneously.  In this firmware the DSP extension accelerates three areas:

- **CRSF channel unpacking** — 11-bit values packed across byte boundaries are extracted using `UXTB16` / `UBFX` in a branchless tight loop.
- **Biquad filter chains** — `SMLAD` / `SMLALD` instructions compute two multiply-accumulates per cycle for Q15 filter implementations via the CMSIS-DSP library path.
- **FFT butterfly stages** — `arm_rfft_fast_f32` uses ARMv8-M SIMD intrinsics for radix-4 butterfly permutations.

### 6.3 CMSIS-DSP Library Integration

The optional FFT vibration analyser (`fft_cmsis.cpp`) uses **`arm_rfft_fast_f32`** from the ARM CMSIS-DSP library, which implements a split-radix real FFT optimised for Cortex-M33:

| Implementation | N=256 compute time | CPU load at 8 Hz |
|---|---|---|
| Naive O(N²) DFT | ≈ 34 000 µs | ~27 % (unusable) |
| CMSIS-DSP `arm_rfft_fast_f32` | ≈ 240 µs | **< 0.2 %** |
| Speedup | **140×** | — |

This makes real-time vibration-frequency tracking feasible on Core 1 without impacting Core 0 at all.

---

## 7. Hardware FIFO — Keeping the Control Loop Uninterrupted

Every time-sensitive peripheral on the RP2354A — UART, SPI, PIO — includes a **hardware FIFO** between the peripheral logic and the CPU.  FIFOs are what allow the flight loop to run without being interrupted by every single byte that arrives or departs.

### 7.1 UART FIFO — Absorbing CRSF Bursts

The ELRS receiver transmits a **26-byte CRSF packet every ~4 ms** at 420 kBaud.  At 420 000 baud each byte arrives every 23.8 µs.  The RP2354A UART0 has a **32-byte RX FIFO**, which means the entire CRSF frame can sit in hardware storage without generating a single interrupt.

```
ELRS Tx module
    │  26 bytes @ 420 kBaud  (≈ 619 µs per frame)
    ▼
┌──────────────────────────────────┐
│  UART0 RX Hardware FIFO (32 B)   │  ← fills automatically, no interrupt
└──────────────────┬───────────────┘
                   │  Core 1 polls opportunistically
                   ▼
  CRSF byte parser  →  16-channel update  →  volatile RC table
                                             (read by Core 0)
```

Core 1 drains the FIFO opportunistically between FFT updates — no tight ISR needed, no Core 0 involvement at all.

### 7.2 SPI FIFO — Non-Blocking Sensor Reads

A 6-byte BMI088 gyro burst read takes **≈ 6 µs** at 10 MHz SPI.  The RP2354A SPI TX and RX FIFOs are each **8 entries × 16 bits** deep.  The ISR pre-loads the entire 7-byte transaction into the TX FIFO, the SPI hardware clocks it out and fills the RX FIFO automatically, and the ISR only checks the RX FIFO at the very end — the CPU is **free to execute other instructions** during the transfer.

### 7.3 PIO TX FIFO — DShot Without CPU Involvement

Each PIO state machine has its own **4-entry TX FIFO**.  After the 1 kHz loop computes motor commands, it writes all four 32-bit DShot frames to the respective PIO FIFOs — **four 32-bit store instructions costing ≈ 8 CPU cycles total**.  The PIO state machines drain their FIFOs independently, clocking out the DShot frames at exactly 600 kbit/s with no further CPU interaction.

If the CPU is momentarily delayed by an ISR, the FIFO absorbs the jitter — the motor signal remains bit-perfect.

> **Why FIFO depth matters for flight control:**  A 1 kHz control loop has exactly 1 000 µs between ticks.  If any peripheral required polling every 24 µs (one CRSF byte at 420 kBaud), the main loop would be interrupted 42 times per millisecond — making deterministic timing impossible.  Hardware FIFOs batch the work into coarse-grained chunks that fit naturally into loop cadences, which is why real-time embedded systems always exploit them when available.

---

## 8. Programmable I/O (PIO) — Precision Motor Signalling

The RP2354A contains **two PIO blocks**, each with **four independent state machines**.  Each state machine is a tiny programmable processor with its own instruction memory, shift registers, and clock divider.  PIO state machines run in parallel with both Cortex-M33 cores and have direct, **cycle-accurate access to GPIO pins** — making them ideal for generating timing-critical waveforms that cannot tolerate software interrupt jitter.

### 8.1 DShot600 on PIO — Architecture

DShot600 requires each bit period to be exactly **1 667 ns** (= 1/600 kHz), with logic-1 pulses high for **1 250 ns** and logic-0 pulses high for **625 ns**.  These tolerances are ±5 %, equivalent to ±83 ns.

A software bit-bang loop on a preemptive scheduler can easily exceed this tolerance.  The PIO state machine solves this by running a dedicated program that produces each bit with **cycle-accurate timing**, completely independent of whatever the Cortex-M33 cores are doing.

```
1 kHz control loop (Core 0):

Compute m1, m2, m3, m4  →  encode DShot frames  →  push to PIO FIFOs
(4 × store instructions, ~8 CPU cycles — Core 0 is now FREE)

PIO0, SM0 (ESC1 / GPIO 1):     PIO0, SM1 (ESC2 / GPIO 4):
┌──────────────────────────┐   ┌──────────────────────────┐
│  pull frame from FIFO    │   │  pull frame from FIFO    │
│  for each of 16 bits:    │   │  (identical program,     │
│   drive GPIO HIGH        │   │   separate clock domain) │
│   delay T1H or T0H       │   │                          │
│   drive GPIO LOW         │   │                          │
│   delay T1L or T0L       │   │                          │
│  inter-frame gap ≥ 2 µs  │   │                          │
└──────────────────────────┘   └──────────────────────────┘

PIO0, SM2 (ESC3 / GPIO 10) and SM3 (ESC4 / GPIO 14): identical.
All four fire simultaneously — start within one PIO clock cycle of each other.
```

### 8.2 Timing Accuracy

The PIO clock divider is set so that each PIO tick = **104.2 ns** (150 MHz ÷ 15.625).  The DShot bit period uses **16 PIO ticks = 1 667 ns**, exactly matching the DShot600 specification.  All four state machines share the same PIO program loaded once into PIO0 instruction memory, so they run in perfect lockstep — all four ESC signals start within **< 7 ns** of each other.

### 8.3 Impact on Dual-Core CPU Loading

| Motor output method | CPU time per 1 kHz update | Notes |
|---|---|---|
| Software bit-bang | ≈ 106.8 µs | Interrupts disabled for 4 motors × 26.7 µs |
| PIO state machines | ≈ 0.027 µs | 4 × FIFO store instruction |
| **Reduction factor** | **≈ 3 960×** | CPU freed for filters, logging, telemetry |

Without PIO, a bit-bang DShot600 driver must disable interrupts for the entire frame duration to prevent jitter — consuming over **10 % of Core 0** just for motor output.  With PIO this drops to effectively **zero**, freeing that headroom for additional biquad filter stages, optional blackbox writes, and MSP telemetry.

---

## 9. Gyro Filter Pipeline

Raw gyroscope data contains two main noise sources: broadband electronic noise from the sensor, and discrete vibration peaks generated by motor and propeller resonance.  The filter chain removes both while preserving the bandwidth needed for responsive attitude control.

```
BMI088 gyro (2 kHz raw)
       │
       ▼
┌─────────────────────────────┐
│  Bias subtraction           │  ← calibrated at boot (2-second still period)
└─────────────┬───────────────┘
              │
              ▼   (if USE_RPM_FILTER)
┌─────────────────────────────┐
│  RPM notch bank             │  8 notches: 4 motors × (fundamental + 2nd harmonic)
│  BLHeli / bidir eRPM input  │  centre frequencies updated every ~4 ms
└─────────────┬───────────────┘
              │
              ▼   (if USE_FFT_ANALYSIS)
┌─────────────────────────────┐
│  Dynamic notch  (1 notch)   │  centre frequency from Core 1 CMSIS-DSP FFT
│  tracks peak vibration freq │  updated every ~50 ms
└─────────────┬───────────────┘
              │
              ▼
┌─────────────────────────────┐
│  Biquad LPF  (per axis)     │  Butterworth 2nd-order, f_c = 150 Hz
│  Direct Form II transposed  │  at 2 kHz sample rate  (numerically stable)
└─────────────┬───────────────┘
              │
              ▼
  Madgwick AHRS  +  PID inner loop
```

Each **biquad filter** is implemented in Direct Form II transposed — the numerically stable form that minimises coefficient-sensitivity error with single-precision floats, important at the low cutoff-to-sample-rate ratio used here (150/2000 = 0.075).

---

## 10. Telemetry, Blackbox, and Safety Systems

### 10.1 USB Telemetry and MSP Protocol

The firmware transmits a **22-field CSV packet at 100 Hz** over USB serial:

```
roll×10, pitch×10, yaw×10,
m1, m2, m3, m4,       (0–1000)
kP, kI, kD,
alt_cm, armed, mode,
volt_mV, curr_cA, bat_pct,
r1rpm, r2rpm, r3rpm, r4rpm,
loop_us, fs_level
```

The same serial port also responds to **MSP v1** (MultiWii Serial Protocol) frames, enabling the official **Betaflight Configurator** to connect and display live attitude, motor test, and PID editing.  The `$M` preamble identifies MSP frames automatically, so both protocols coexist without configuration.

A **Processing 4 GUI** (`gui/mini_bf_config.pde`) provides a four-tab interface:

| Tab | Contents |
|---|---|
| Flight | 3-D orientation cube, motor output bars + ESC RPM, battery panel, FFT spectrum |
| PID Tuning | Roll/Pitch and Yaw sliders with live FC feedback; tuning tips |
| RC Calibration | Per-channel expo and deadband adjustment; START/END sweep buttons |
| Self-Test | Live hardware verification with pass/fail badges |

### 10.2 Blackbox Flight Data Recorder *(optional — requires external W25Q128)*

> **Hardware note:** The W25Q128 SPI flash is **not populated** on the current RB-RP2354A board revision.  The blackbox subsystem is fully implemented in firmware and can be enabled by soldering a W25Q128JV to the SPI pads (CS = GPIO 28) and adding `#define USE_BLACKBOX` to `config.h`.  All description below applies when the flash is present.

When fitted, compact **32-byte binary frames** are written to the W25Q128 SPI flash at **1 kHz** during armed flight.  Each frame captures:

```c
uint32_t timestamp_us;
int16_t  roll_x10, pitch_x10, yaw_x10;    // degrees × 10
int16_t  gx_x100, gy_x100, gz_x100;       // rad/s × 100
uint16_t m1, m2, m3, m4;                  // motor output 0–1000
uint16_t altitude_cm;
uint16_t voltage_mV;
uint8_t  armed_mode;
uint8_t  flags;
uint16_t loop_time_us;
// total: 32 bytes
```

**Page-write buffering** (8 frames accumulated before a 256-byte flash page program) keeps write latency below 700 µs.  At 1 kHz the W25Q128 holds ≈ **524 seconds** of flight data per session.  A Python decoder script (`blackbox/decode.py`) converts raw dumps to CSV for analysis in Excel or pandas.

```bash
# Dump flash over USB
python3 blackbox/dump_flash.py /dev/ttyACM0 flash.bin

# Decode all sessions to CSV
python3 blackbox/decode.py flash.bin
```

### 10.3 Four-Level Failsafe

The firmware monitors CRSF link age and battery voltage continuously:

| Level | Trigger | Response |
|---|---|---|
| `NONE` | All nominal | Normal flight |
| `WARN` | Link lost > 100 ms or battery low | Alert only |
| `LAND` | Link lost > 500 ms or battery critical | Throttle ramp to zero over 8 s |
| `KILL` | Link lost > 2 s or landing complete | Immediate motor stop + DISARM |

### 10.4 Hardware Self-Test at Boot

Before the flight loop starts, a hardware self-test verifies:

1. BMI088 accelerometer chip ID over SPI (expect `0x1E`)
2. BMI088 gyroscope chip ID over SPI (expect `0x0F`)
3. BMP580 barometer chip ID over I²C (expect `0x50`)
4. W25Q128 flash JEDEC ID over SPI (expect `0xEF4018`) — *skipped if `USE_BLACKBOX` is not defined*
5. Battery voltage within safe range (3.0 V – 25.2 V)
6. CRSF link received within 2 s
7. Gyro RMS < 0.05 rad/s (board stationary)
8. Accelerometer magnitude 0.90 – 1.10 g (board level)

Results are printed to Serial and parsed by the GUI Self-Test tab.

---

## 11. Performance Summary

| Subsystem | Rate | Core | CPU Cost |
|---|---|---|---|
| BMI088 gyro DRDY ISR | 2 000 Hz | Core 0 | < 15 µs |
| Gyro filters (bias + RPM notch + LPF) | 2 000 Hz | Core 0 | < 26 µs |
| Madgwick AHRS (FPU hardware) | 1 000 Hz | Core 0 | < 1 µs |
| Dual PID — 3 axes, inner + outer loop | 1 000 Hz | Core 0 | < 12 µs |
| Altitude fusion + hold | 1 000 Hz | Core 0 | < 8 µs |
| DShot600 PIO (4 motors simultaneous) | 1 000 Hz | Core 0 | < 0.1 µs |
| Blackbox page write *(optional)* | ~125 Hz | Core 0 | < 700 µs |
| USB telemetry (22-field CSV) | 100 Hz | Core 0 | < 15 µs |
| CRSF parser — 16 channels | async | Core 1 | < 5 % |
| CMSIS-DSP FFT — N = 256 | ~8 Hz | Core 1 | < 0.2 % |
| **Total — Core 0** | — | — | **< 15 %** |
| **Total — Core 1** | — | — | **< 5 %** |

---

## 12. Quick-Start Guide

### 12.1 Build Environment

1. Install **Arduino IDE 2.3.8**
2. Add board package URL:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
3. Boards Manager → search **Raspberry Pi Pico/RP2040** → install latest ≥ 3.x
4. Board: **Raspberry Pi Pico 2** · CPU speed: **150 MHz**
5. Open `RP2354A_FC.ino`

### 12.2 Motor Type Selection

Edit `config.h` and uncomment **exactly one** line:

```c
#define MOTOR_TYPE_BRUSHED   // coreless motors via N-MOSFET
//#define MOTOR_TYPE_BLDC    // BLDC motors via DShot600 ESC
```

### 12.3 Optional Features

```c
#define USE_BLACKBOX        // W25Q128 flight logging — requires flash fitted to GPIO 28 CS pad
#define USE_BATTERY_MON     // ADC voltage sense (GPIO 27)
#define USE_RPM_FILTER      // per-motor vibration notch (needs ESC telemetry)
#define USE_FFT_ANALYSIS    // CMSIS-DSP vibration spectrum on Core 1
// In motors/dshot_pio.h:
#define DSHOT_BIDIR         // bidirectional eRPM from ESC
```

### 12.4 First Boot Sequence

1. Flash firmware.  Open Serial Monitor at **115 200 baud**.
2. Place board **flat and still** — 2-second IMU gyro calibration runs automatically.
3. Self-test prints `PASS` / `FAIL` for all eight hardware checks.
4. Open **Processing 4 GUI** (`gui/mini_bf_config.pde`) for live attitude cube and PID tuning.
5. For **Betaflight Configurator**: connect on the same COM/tty port — MSP v1 is active alongside CSV telemetry.

### 12.5 USB Command Reference

| Command | Effect |
|---|---|
| `ARM` / `DISARM` | Arm or disarm motors |
| `MODE_ACRO` / `MODE_ANGLE` | Switch flight mode |
| `CALIB` | Start 2-second IMU calibration (disarmed only) |
| `SET_KP:<val>` / `SET_KI:<val>` / `SET_KD:<val>` | Set roll + pitch PID gains |
| `SET_YAW_KP/KI/KD:<val>` | Set yaw gains independently |
| `RC_CAL_START` / `RC_CAL_END` | RC sweep calibration |
| `SET_EXPO:<ch>:<val>` | Set stick expo curve (0.0–1.0) per channel |
| `SET_DEADBAND:<ch>:<val>` | Set stick deadband (0.0–0.20) per channel |
| `ESC_CAL_START` | ESC PWM range calibration (**remove props first**) |
| `MOTOR_TEST:<m>:<pct>` | Spin motor 0–3 at pct % (max 15 %) |
| `MOTOR_TEST_STOP` | Stop motor test |
| `SELF_TEST` | Run hardware self-test, stream structured results |
| `DUMP_BB` | Stream W25Q128 flash over serial — *(requires flash fitted)* |
| `ERASE_BB` | Chip-erase W25Q128, disarmed only — *(requires flash fitted)* |

---

## Hardware Pinout

| Signal | GPIO | Notes |
|---|---|---|
| SPI MOSI | 19 | BMI088 shared bus; also W25Q128 if flash fitted |
| SPI MISO | 20 | |
| SPI SCK | 22 | |
| Accel CS | 21 | BMI088 accelerometer |
| Gyro CS | 23 | BMI088 gyroscope |
| Gyro DRDY | 24 | Rising-edge interrupt, 2 kHz |
| I²C SDA | 16 | BMP580 barometer |
| I²C SCL | 17 | |
| Brushed M1 | 29 | MOSFET PWM |
| Brushed M2 | 11 | |
| Brushed M3 | 18 | |
| Brushed M4 | 25 | |
| BLDC ESC1 | 1 | DShot600 PIO |
| BLDC ESC2 | 4 | |
| BLDC ESC3 | 10 | |
| BLDC ESC4 | 14 | |
| ELRS TX | 12 | UART0, 420 000 baud CRSF |
| ELRS RX | 13 | |
| Battery Vsense | 27 | ADC1, voltage divider 100 kΩ / 10 kΩ |
| Flash CS | 28 | W25Q128 blackbox CS — *(pad reserved; flash not fitted on current revision)* |

---

## ELRS Channel Map

| CH | Function | Range |
|---|---|---|
| 1 | Roll | ±1.0 (after calibration + expo) |
| 2 | Pitch | ±1.0 |
| 3 | Throttle | 0.0 – 1.0 |
| 4 | Yaw | ±1.0 |
| 5 | Arm switch | > 1700 µs = ARM, < 1300 µs = DISARM |
| 6 | Mode switch | > 1500 µs = ANGLE, ≤ 1500 µs = ACRO |
| 7 | Altitude hold | > 1500 µs = ON |

---

## License

MIT — free for educational and personal use.

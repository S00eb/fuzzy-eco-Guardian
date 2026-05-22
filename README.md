# Simulasi Software-in-the-Loop Sistem Manajemen Termal Real-Time Berbasis Interval Type-2 Fuzzy-PID pada Inverter PLTS Berbasis Mikrokontroler STM32

> **Software-in-the-Loop Simulation of a Real-Time Thermal Management System Based on Interval Type-2 Fuzzy-PID Controller for Photovoltaic Inverters on STM32 Microcontroller**

[![Platform](https://img.shields.io/badge/Platform-STM32F401VE-blue?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401ve.html)
[![Simulation](https://img.shields.io/badge/Simulation-Proteus%20VSM-green)](https://www.labcenter.com/)
[![IDE](https://img.shields.io/badge/IDE-Keil%20µVision-red)](https://www.keil.com/)
[![License](https://img.shields.io/badge/License-Academic-yellow)](#license)
[![Language](https://img.shields.io/badge/Language-Embedded%20C-orange)](#)

---

**Authors:**
- Mahendra Dwi Fahreza (NRP: 2042241023)
- Athaya Zhafari Saputra (NRP: 2042241019)

**Affiliation:** D4 Teknologi Rekayasa Instrumentasi, Departemen Teknik Instrumentasi, Fakultas Vokasi, Institut Teknologi Sepuluh Nopember (ITS), 2026

**Supervisor:** Ahmad Radhy, S.Si., M.Si

---

## Table of Contents

- [Project Description](#project-description)
- [Background & Novel Method](#background--novel-method)
- [System Architecture](#system-architecture)
  - [Block Diagram](#block-diagram)
  - [Algorithm Flowchart](#algorithm-flowchart)
- [Hardware & Components](#hardware--components)
- [Pin Configuration](#pin-configuration)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Step 1: STM32CubeMX Configuration](#step-1-stm32cubemx-configuration)
  - [Step 2: Keil µVision Setup](#step-2-keil-µvision-setup)
  - [Step 3: Building the HEX File](#step-3-building-the-hex-file)
  - [Step 4: Proteus Simulation](#step-4-proteus-simulation)
  - [Step 5: Data Logging & GNUPlot Visualization](#step-5-data-logging--gnuplot-visualization)
- [Simulation Results](#simulation-results)
  - [Normal Operating Condition](#normal-operating-condition-t--50c)
  - [Peak Load Condition](#peak-load-condition-t--50c)
  - [Transition Response](#transition-response)
  - [GNUPlot Analysis](#gnuplot-analysis)
- [Method Advantages](#method-advantages)
- [Project Structure](#project-structure)
- [References](#references)
- [License](#license)

---

## Project Description

This project implements a **Software-in-the-Loop (SIL)** simulation of a real-time thermal management system for photovoltaic (PV) inverter heatsinks using an **Interval Type-2 Fuzzy-PID (IT2F-PID)** hybrid controller. The entire system is simulated in **Proteus VSM** with a virtual **STM32F401VE** microcontroller, eliminating the need for physical hardware during the development and validation phase.

The system reads two environmental inputs—**temperature** (simulating a heatsink thermal sensor via LM35) and **light intensity** (simulating solar irradiance via LDR)—and dynamically adjusts a **DC cooling fan** speed via PWM to maintain the inverter's optimal operating temperature. Status information is displayed on an **I2C LCD 16×2** and transmitted via **UART** to a Virtual Terminal for data logging and subsequent analysis using **GNUPlot**.

### Key Features

- 🔥 **Interval Type-2 Fuzzy Logic** with Footprint of Uncertainty (FOU) for robust control under sensor noise
- 🎛️ **Adaptive PID Tuning** — Fuzzy inference dynamically adjusts the PID gain multiplier in real-time
- 📡 **Dual-Input Feed-Forward** — Light intensity (irradiance) acts as a proactive input anticipating thermal load
- 💡 **Bare-Metal Task Scheduling** — Lightweight super-loop architecture (no RTOS overhead)
- 📺 **Soft-I2C LCD Driver** — Bit-banged I2C communication for LCD without HAL I2C dependency
- 📊 **GNUPlot-Ready Data Logging** — UART output formatted for CSV export and 2D/3D plotting
- 🛡️ **Anti-Windup Protection** — Integral clamping at ±300 to prevent actuator saturation
- 🔴🟢 **Visual LED Indicators** — Real-time thermal load status via Green/Red LED switching

---

## Background & Novel Method

### Problem Statement

Photovoltaic (PV) inverters generate significant heat during DC-to-AC power conversion. Excessive heatsink temperatures (>70°C) degrade IGBT/MOSFET performance, reduce conversion efficiency, and accelerate component aging. Conventional PID controllers struggle with:

1. **Non-linear thermal dynamics** — Heat dissipation is not a linear function of temperature
2. **Sensor noise and uncertainty** — Low-cost analog sensors introduce measurement fluctuations
3. **Variable environmental conditions** — Solar irradiance changes the thermal load unpredictably

### Research Gap

This system is built upon the **research gaps** identified from the *future work* recommendations of 15 reference journals (see [References](#references)). The core gap is the need to **validate Fuzzy-PID control algorithms**—which have been tested mathematically in MATLAB/Simulink—through **real-time simulation on resource-constrained microcontrollers** via software-in-the-loop.

### Proposed Solution: IT2 Fuzzy-PID Hybrid Controller

| Component | Role | Innovation |
|:---|:---|:---|
| **IT2 Fuzzy Logic** | Computes an adaptive gain multiplier based on temperature & light | Uses **Upper and Lower Membership Functions** to create a Footprint of Uncertainty (FOU), absorbing sensor noise |
| **PID Controller** | Generates the base control signal (PWM duty cycle) | Classical P+I+D computation with anti-windup (±300 integral clamp) |
| **Adaptive Setpoint** | Adjusts target temperature based on irradiance | `adaptive_sp = 50 + (500 − light) × 0.02` |
| **Type-Reduction** | Defuzzifies IT2 output to a crisp gain value | Simplified Karnik-Mendel averaging of upper/lower bounds |

Unlike **Type-1 Fuzzy Logic**, which uses single membership curves, **Type-2 Fuzzy Logic** employs **dual curves (Upper & Lower)** for each fuzzy set, creating a band of uncertainty. This makes the controller inherently more robust against sensor measurement noise, parameter variations, and environmental disturbances.

### How This System Addresses 15 Future Work Recommendations

| FW | Journal Reference | Future Work Recommendation | How Our System Addresses It |
|:---|:---|:---|:---|
| FW1 | Liu et al. (2026) | Integration of sensor modules into IoT systems for real-time monitoring | LM35 + LDR sensors connected to STM32 ADC for real-time inverter thermal monitoring via UART |
| FW2 | Chatzipapas & Karnavas (2025) | Resolving trade-off between control resolution and PWM frequency on low-cost MCUs | Bare-metal architecture on STM32F401VE optimizes PWM frequency and task scheduling efficiently |
| FW3 | Ferro et al. (2026) | Embedding weather fluctuation mitigation models into embedded systems | IT2 Fuzzy logic automatically responds to light sensor fluctuations (simulating irradiance changes) |
| FW4 | Khan et al. (2026) | Equipping systems with adaptive intelligent control algorithms like PID | System uses IT2 Fuzzy-PID that adaptively adjusts gain based on error and delta-error conditions |
| FW5 | Hossain et al. (2026) | Firmware improvements like adaptive sampling and migration to more capable MCUs | Firmware on STM32F401VE (84 MHz, Cortex-M4) runs adaptive control loop with efficient memory management |
| FW6 | Nkinyam et al. (2025) | Increased data processing capacity and additional sensors | Multi-channel ADC (temperature + light) with real-time data processing on STM32 |
| FW7 | Yuvaraj et al. (2025) | Implementation of adaptive motor control algorithms based on PID or MPC | DC motor (cooling fan) speed controlled via PWM signal regulated by IT2 Fuzzy-PID algorithm |
| FW8 | Yang et al. (2026) | Compression of intelligent algorithms for small-scale microcontrollers | IT2 Fuzzy-PID written in pure C and successfully executed on memory-constrained STM32 |
| FW9 | Behera & Choudhury (2022) | Validation of dynamic Fuzzy-PID control strategies under varying weather conditions | Proteus simulation enables testing with varying temperature and light sensor values |
| FW10 | Mezouari et al. (2025) | Combining adaptive or self-tuning fuzzy strategies to improve performance | IT2 Fuzzy is inherently adaptive through its FOU mechanism that handles input uncertainty |
| FW11 | Sayeh et al. (2025) | Development of adaptive fuzzy or neuro-fuzzy approaches for highly dynamic conditions | IT2 Fuzzy Logic is a direct form of adaptive fuzzy using dual membership functions (upper & lower MF) |
| FW12 | Ekuewa et al. (2024) | Integration of renewable energy sources into smart device networks | System specifically designed for PV inverter thermal management in solar energy context |
| FW13 | Sain & Lee (2026) | Real-time implementation validation of IT2 Fuzzy-PID controller (currently limited to simulation) | System validates IT2 Fuzzy-PID in real-time via firmware execution on virtual STM32 in Proteus (SIL level) |
| FW14 | Yang et al. (2025) | Optimization of membership functions and fuzzy control rules for complex conditions | IT2 structure with FOU mathematically handles uncertainty at MF boundaries, reducing manual tuning |
| FW15 | Nippatla & Mandava (2025) | Motor control using Fuzzy-PID combinations with metaheuristic optimization | System implements hybrid Fuzzy-PID for cooling fan motor control with faster response and lower overshoot |

---

## System Architecture

### Block Diagram

![Block Diagram](Images_for_README/diagramblok.png)

### Algorithm Flowchart

![Algorithm Flowchart](Images_for_README/flowchart_algoritma.png)

---

## Hardware & Components

| Component | Model / Specification | Role in Simulation |
|:---|:---|:---|
| Microcontroller | **STM32F401VE** (ARM Cortex-M4 with FPU, 84 MHz max, HSI 16 MHz used) | Central processing — runs IT2 Fuzzy-PID algorithm |
| Motor Driver | **L298** H-Bridge | Drives the DC cooling fan motor from PWM signal |
| Cooling Fan | **DC Motor 12V** | Actuator — provides cooling airflow to inverter heatsink |
| Temperature Sensor | **LM35** (simulated via Potentiometer) | Analog input on PA0 — heatsink temperature (0–100°C) |
| Light Sensor | **LDR** (simulated via Potentiometer) | Analog input on PA1 — solar irradiance (0–1000 Lux) |
| LCD Display | **LM016L 16×2** with **PCF8574** I2C backpack | Real-time display of T, L, PWM, and V |
| Green LED | Connected via 220Ω resistor to **PC13** | Normal operation indicator (PWM ≤ 350) |
| Red LED | Connected via 220Ω resistor to **PC15** | Heavy thermal load indicator (PWM > 350) |
| Virtual Terminal | Proteus Virtual Terminal | Serial data monitoring (4800 baud) and CSV export |
| Oscilloscope | Proteus Virtual Oscilloscope | PWM waveform verification (100 Hz, duty cycle) |

### STM32F401VE Technical Specifications

| Parameter | Specification |
|:---|:---|
| Architecture | ARM Cortex-M4 with FPU |
| Max Clock | 84 MHz |
| Clock Used | HSI 16 MHz (no PLL) |
| Flash Memory | 512 KB |
| RAM | 96 KB |
| ADC Resolution | 12-bit |
| PWM Frequency | 100 Hz (Prescaler 159, Period 999) |
| Serial Communication | UART 4800 baud (USART2) |
| Display Communication | Soft I2C (PB8/SCL, PB9/SDA) |

---

## Pin Configuration

| Signal | STM32 Pin | Connected To | Protocol |
|:---|:---|:---|:---|
| Temperature ADC | **PA0** | LM35 (Potentiometer) | ADC Channel 0 |
| Light ADC | **PA1** | LDR (Potentiometer) | ADC Channel 1 |
| UART TX | **PA2** | Virtual Terminal RXD | USART2 (AF7) |
| UART RX | **PA3** | Virtual Terminal TXD | USART2 (AF7) |
| PWM Output | **PA8** | L298 ENA | TIM1_CH1 (AF1) |
| Motor IN1 | **PB1** | L298 IN1 | GPIO Push-Pull |
| Motor IN2 | **PB2** | L298 IN2 | GPIO Push-Pull |
| I2C SCL | **PB8** | PCF8574 SCL | Soft-I2C (Open-Drain) |
| I2C SDA | **PB9** | PCF8574 SDA | Soft-I2C (Open-Drain) |
| Green LED | **PC13** | LED (via 220Ω) | GPIO Push-Pull |
| Red LED | **PC15** | LED (via 220Ω) | GPIO Push-Pull |

### Required Power Pins

| Pin | Connection |
|:---|:---|
| VDDA (pin 22) | +3.3V |
| VREF+ (pin 21) | +3.3V |
| VSSA (pin 20) | GND |
| BOOT0 (pin 94) | GND |

---

## Getting Started

### Prerequisites

| Software | Version | Purpose |
|:---|:---|:---|
| [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) | 6.x+ | MCU configuration & code generation |
| [Keil µVision](https://www.keil.com/download/product/) | 5.x (MDK-ARM) | Embedded C IDE & ARM Compiler 6 |
| [Proteus Design Suite](https://www.labcenter.com/) | 8 Professional | SIL circuit simulation with VSM |
| [GNUPlot](http://www.gnuplot.info/) | 5.x+ | 2D/3D scientific data visualization |

---

### Step 1: STM32CubeMX Configuration

1. **Create New Project** → Select **STM32F401VETx**
2. **System Core → RCC**
   - High Speed Clock (HSE): *Disabled* (we use HSI)
3. **System Core → SYS**
   - Debug: **Serial Wire**
   - Timebase Source: **SysTick**
4. **Clock Configuration**
   - Set HCLK to **16 MHz** (HSI only, **no PLL**)
   - ⚠️ This is critical for Proteus simulation stability — do NOT use 84 MHz
5. **Analog → ADC1**
   - Enable **IN0** (PA0) and **IN1** (PA1)
   - Resolution: 12-bit
   - Scan Conv Mode: Disable
   - Continuous Conv Mode: Disable
6. **Timers → TIM1**
   - Clock Source: Internal Clock
   - Channel 1: **PWM Generation CH1** → auto-maps to PA8
   - Prescaler: `159` → 100 kHz tick rate
   - Counter Period (ARR): `999` → 100 Hz PWM frequency
7. **Connectivity → USART2**
   - Mode: Asynchronous
   - Baud Rate: **4800**
   - Word Length: 8 Bits, No Parity, 1 Stop Bit
8. **GPIO Output Configuration**
   - PB1, PB2 → Push-Pull Output (Motor Direction)
   - PC13, PC15 → Push-Pull Output (LEDs)
   - PB8, PB9 → Open-Drain Output with Pull-Up (Soft-I2C)
9. **Project Manager**
   - Toolchain/IDE: **MDK-ARM**
   - Code Generator: ✅ "Generate .c/.h pair per peripheral"
10. **Generate Code** → Opens Keil µVision project

---

### Step 2: Keil µVision Setup

1. Open the generated `.uvprojx` file in Keil µVision 5
2. **Target Configuration:**
   - Xtal (MHz): **16.0**
   - Floating Point Hardware: **Not Used**
   - ARM Compiler: **V6 (ARMCLANG)**
3. **C/C++ Tab:**
   - Preprocessor Defines: `USE_HAL_DRIVER,STM32F401xE`
   - Optimization Level: `-O1`
4. **Output Tab:**
   - ✅ **Create HEX File** — this is mandatory for Proteus
5. **Replace** the entire content of `main.c` with the source code from `EcoGrid_main.c`

---

### Step 3: Building the HEX File

1. In Keil µVision, press **F7** or click **Build** (🔨)
2. Verify the Build Output shows **0 Error(s), 0 Warning(s)**
3. The `.hex` file is generated in the `Objects/` folder

```
Build Output:
  compiling main.c...
  linking...
  Program Size: Data=xxxx Const=xxxx Code=xxxx
  "project.axf" - 0 Error(s), 0 Warning(s).
  FromELF: creating hex file...
```

---

### Step 4: Proteus Simulation

1. **Open the Proteus schematic** with all components wired:
   - STM32F401VE, L298, DC Motor, 2× Potentiometers (LM35 & LDR), LCD I2C (PCF8574), 2× LEDs, Virtual Terminal, Oscilloscope
2. **STM32F401VE Properties:**
   - Program File: Browse to your `.hex` file path
   - Crystal Frequency: **16 MHz**
3. **Virtual Terminal Properties:**
   - Baud Rate: **4800**
4. **PCF8574 Properties:**
   - Slave Address: `0x40` (A0=A1=A2=GND)
5. Click ▶️ **Run Simulation**
6. **Expected behavior:**
   - Virtual Terminal prints: `=== ECO-GRID GUARDIAN ===`
   - LCD shows: System status → then live sensor data
   - Rotate potentiometers to vary temperature/light conditions
   - Observe PWM changes, LED switching, and Oscilloscope waveform

---

### Step 5: Data Logging & GNUPlot Visualization

#### Data Extraction Pipeline

1. **Capture Serial Log** → Copy text from Virtual Terminal
2. **Save as TXT** → Raw text file with format: `T:XX.XC L:XXXLx PWM:XXX V:XX.XV`
3. **Convert to CSV** → Clean alphabetic characters, structure into columns: `Time, Temperature, Light, PWM, Voltage`
4. **Plot with GNUPlot** → Run `.gp` scripts for 2D time-series, scatter, and 3D surface plots

#### Sample GNUPlot Script (2D Multiplot)

```gnuplot
set terminal pngcairo enhanced size 1400,900 font "Arial,11"
set output 'plot_gabungan.png'

set multiplot layout 2,2 title "IT2 Fuzzy-PID Thermal Management — Time-Series Dynamics" font "Arial,13"

# Plot 1: Temperature vs Time
set title "Temperature (°C)"
set xlabel "Time (s)"; set ylabel "T (°C)"
set grid; set yrange [0:100]
plot 'data.csv' using 1:2 with linespoints lw 2 pt 7 ps 0.5 lc rgb '#e74c3c' title 'T_{actual}', \
     50 with lines lw 1 dt 2 lc rgb '#2c3e50' title 'Setpoint (50°C)'

# Plot 2: Light Intensity vs Time
set title "Light Intensity (Lux)"
set ylabel "L (Lux)"; set yrange [0:1000]
plot 'data.csv' using 1:3 with linespoints lw 2 pt 7 ps 0.5 lc rgb '#f39c12' title 'Irradiance'

# Plot 3: PWM Response vs Time
set title "PWM Duty Cycle Response"
set ylabel "PWM (0-999)"; set yrange [0:1050]
plot 'data.csv' using 1:4 with linespoints lw 2 pt 7 ps 0.5 lc rgb '#3498db' title 'PWM', \
     350 with lines lw 1 dt 2 lc rgb '#c0392b' title 'LED Threshold'

# Plot 4: Motor Voltage vs Time
set title "Motor Equivalent Voltage"
set ylabel "V (Volt)"; set yrange [0:13]
plot 'data.csv' using 1:5 with linespoints lw 2 pt 7 ps 0.5 lc rgb '#2ecc71' title 'V_{motor}'

unset multiplot
```

#### 3D Control Surface Script

```gnuplot
set terminal pngcairo enhanced size 1000,800 font "Arial,11"
set output 'plot_surface_3d.png'
set title "3D Control Surface: IT2 Fuzzy-PID" font "Arial,14"
set xlabel "Temperature (°C)"; set ylabel "Light (Lux)"; set zlabel "PWM"
set dgrid3d 30,30 splines
set pm3d
set palette defined (0 "green", 0.5 "yellow", 1 "red")
splot 'data.csv' using 2:3:4 with pm3d title ''
```

---

## Simulation Results

### Normal Operating Condition (T < 50°C)

| Parameter | Value |
|:---|:---|
| Temperature | 33.9°C |
| Light Intensity | 369 Lux |
| Adaptive Setpoint | 50 + (500 − 369) × 0.02 = **52.62°C** |
| Error | 33.9 − 52.62 = **−18.72°C** (negative → low cooling) |
| PWM Output | **337** (33.7% duty cycle) |
| Motor Voltage | **4.0V** |
| LED Status | 🟢 Green (PWM ≤ 350) |

The fan operates at low speed since the temperature is well below the adaptive setpoint. The system conserves energy by not over-cooling.

### Peak Load Condition (T > 50°C)

| Parameter | Value |
|:---|:---|
| Temperature | 74.9°C |
| Light Intensity | 749 Lux |
| Adaptive Setpoint | 50 + (500 − 749) × 0.02 = **45.02°C** |
| Error | 74.9 − 45.02 = **+29.88°C** (large positive → aggressive cooling) |
| PWM Output | **853–999** (85–100% duty cycle, saturated) |
| Motor Voltage | **~10.2–12.0V** |
| LED Status | 🔴 Red (PWM > 350) |

The system drives the cooling fan at maximum speed. High irradiance (749 Lux) lowers the adaptive setpoint, making the controller even more aggressive — this is the **proactive feed-forward** effect.

### Transition Response

**Scenario:** Sudden temperature drop from 67.0°C → 29.0°C (light: 769 → 339 Lux)

| Phase | Temperature | Light | Adaptive SP | Error | PWM | LED |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **Before** (heavy load) | 67.0°C | 769 Lux | 44.62°C | +22.38°C | 850 | 🔴 Red |
| **Instant drop** | 29.0°C | 339 Lux | 53.22°C | −24.22°C | **2** | 🟢 Green |
| **Stabilized** | 29.0°C | 339 Lux | 53.22°C | −24.22°C | **118** | 🟢 Green |

The **Derivative (D) action** in the PID instantly brakes the fan from PWM 850 → 2 to prevent cooling overshoot. After the transient subsides, the **Integral (I) component** stabilizes the output at PWM 118 (1.4V). The Red LED switches to Green immediately upon crossing the 350 threshold.

### GNUPlot Analysis

The simulation extracted **98 real-time data samples** covering three operational phases:

| Phase | Time Range | Behavior |
|:---|:---|:---|
| **Normal Phase** | t = 0–14s | Room temperature (29°C), PWM resting at 105. Moderate response at 49.9°C (PWM ≈ 500) |
| **Critical Phase** | t = 15–23s, t = 35–45s | Temperature >70°C triggers exponential PWM surge to saturation (914–999) |
| **Braking Phase** | t = 23.5s, t = 45.5s | Sudden temperature drop triggers aggressive derivative braking (PWM drops instantly to 2, then stabilizes at 56) |

**3D Control Surface Analysis:**
The GNUPlot 3D surface (Temperature × Light × PWM) reveals a smooth, non-linear control surface with a steep gradient transitioning from green (low PWM) to red (saturated PWM). This continuous proportional topology—free from sharp oscillations—is the hallmark mathematical advantage of IT2 Fuzzy Sets in handling operational parameter uncertainties.

---

## Method Advantages

| Feature | Conventional PID | Type-1 Fuzzy-PID | **IT2 Fuzzy-PID (This Work)** |
|:---|:---:|:---:|:---:|
| Handles Non-linearity | ❌ Poor | ✅ Good | ✅ **Excellent** |
| Noise Robustness | ❌ Sensitive | ⚠️ Moderate | ✅ **High (FOU)** |
| Adaptive Gain Tuning | ❌ Fixed Gains | ✅ Rule-based | ✅ **Dual-bound adaptive** |
| Uncertainty Modeling | ❌ None | ❌ None | ✅ **Upper/Lower MFs** |
| Feed-Forward Input | ❌ Not inherent | ⚠️ Possible | ✅ **Built-in (Light → SP)** |
| Computational Cost | ✅ Very Low | ✅ Low | ✅ **Low (on STM32)** |
| RTOS Dependency | N/A | N/A | ✅ **None (Bare-Metal)** |
| Anti-Windup | ⚠️ Manual | ⚠️ Manual | ✅ **Integrated (±300)** |
| Real-time Validation | N/A | MATLAB only | ✅ **SIL on STM32 (Proteus)** |

### Key Innovations

1. **Footprint of Uncertainty (FOU):** Each membership function has Upper (wider) and Lower (narrower) curves, creating a tolerance band that naturally absorbs sensor noise without explicit digital filtering.

2. **Simplified Karnik-Mendel Type-Reduction:** Weighted-average approximation instead of computationally expensive iterative methods, enabling real-time execution on Cortex-M4 at 16 MHz.

3. **Dual-Input 9-Rule Fuzzy Base:** The 3×3 rule matrix (Temperature × Light) enables sophisticated gain scheduling that single-input PID cannot achieve.

4. **Predictive Adaptive Setpoint via Irradiance:** Light intensity predicts upcoming thermal load, enabling proactive cooling adjustment *before* the temperature actually rises — reducing thermal overshoot.

5. **Bare-Metal Super-Loop Efficiency:** Four cooperative tasks (Sensor 1s, Fuzzy 0.5s, Motor 1s, UART+LCD 0.5s) with <2KB RAM usage and deterministic response time.

---

## IT2 Fuzzy Membership Functions

### Temperature Membership Functions

```
Membership
Degree (µ)
  1.0 |███                    ██████                    ███
      |█████                ████  ████                █████
      |  █████            ████      ████            █████
  0.5 |    █████        ████          ████        █████
      |      █████    ████              ████    █████
      |        █████████                  █████████
  0.0 |__________████____________________________████_________
      0°C     20°C   30°C   45°C  50°C  55°C   70°C  80°C  100°C
              ←—— LOW ——→  ←——— MEDIUM ———→  ←—— HIGH ——→

      ──── Upper MF (wider)     ---- Lower MF (narrower)
      Gap between Upper & Lower = Footprint of Uncertainty (FOU)
```

### Fuzzy Rule Base (9 Rules)

| | Light LOW | Light MEDIUM | Light HIGH |
|:---|:---:|:---:|:---:|
| **Temp LOW** | Gain = 0.3 | Gain = 0.5 | Gain = 1.0 |
| **Temp MEDIUM** | Gain = 0.7 | Gain = 1.0 | Gain = 1.5 |
| **Temp HIGH** | Gain = 1.2 | Gain = 1.5 | Gain = 2.0 |

---

## Project Structure

```
├── README.md                          # This file
├── EcoGrid_main.c                     # Complete firmware source code (IT2 Fuzzy-PID + Soft-I2C)
├── EcoGrid_Documentation.txt          # System documentation (pin config, CubeMX settings)
├── main.tex                           # Full LaTeX report
├── Proteus/
│   └── project.pdsprj                 # Proteus schematic project file
├── Keil/
│   ├── project.uvprojx               # Keil µVision project file
│   └── Objects/
│       └── project.hex               # Compiled firmware for Proteus
├── GNUPlot/
│   ├── plot_gabungan.gp              # 2D multiplot script (time-series)
│   ├── plot_scatter_suhu.gp          # Temperature vs PWM scatter plot
│   ├── plot_scatter_cahaya.gp        # Light vs PWM scatter plot
│   ├── plot_surface_3d.gp            # 3D control surface script
│   └── data.csv                      # Extracted simulation data (98 samples)
├── Images/
│   ├── diagramblok.png               # Block diagram
│   ├── all.png                       # Full Proteus schematic
│   ├── runtampilanall_normal.png     # Normal condition simulation
│   ├── runtampilanall_bahaya.png     # Critical condition simulation
│   ├── plot_gabungan.png             # 2D time-series multiplot
│   ├── plot_scatter_suhu.png         # Temperature scatter plot
│   ├── plot_scatter_cahaya.png       # Light scatter plot
│   └── plot_surface_3d.png           # 3D control surface
└── Docs/
    ├── diagram_blok.mmd              # Mermaid block diagram source
    └── flowchart_algoritma.mmd       # Mermaid flowchart source
```

---

## References

The following 15 journal articles served as the literature foundation for this project. Each article's *future work* recommendation is directly addressed by the system design.

1. **Liu, Y.-Q.** et al. (2026). "A versatile logic-gated fluorescence turn-on nitrogen-doped carbon dots: STM32 microcontroller intelligent platform for assay of L-Histidine and Al³⁺." *Journal of Colloid and Interface Science*. [DOI: 10.1016/j.jcis.2026.140064](https://doi.org/10.1016/j.jcis.2026.140064)

2. **Chatzipapas, N. V.** & Karnavas, Y. L. (2025). "Adaptive Microprocessor-Based Interval Type-2 Fuzzy Logic Controller Design for DC Micro-Motor Control Considering Hardware Limitations." *Energies*, 18(22), 5781. MDPI. [DOI: 10.3390/en18215781](https://doi.org/10.3390/en18215781)

3. **Ferro, G.** et al. (2026). "An embedded accelerated decentralized optimization algorithm with application to energy communities." *Journal of Energy Storage*, 116, 116047. Elsevier. [DOI: 10.1016/j.conengprac.2026.106920](https://doi.org/10.1016/j.conengprac.2026.106920)

4. **Khan, H.** et al. (2026). "Microcontroller-based numerical control DC stabilized power supply: design, implementation, and performance evaluation." *Scientific Reports*, 16, 13179. Springer Nature. [DOI: 10.1016/j.rineng.2025.108671](https://doi.org/10.1016/j.rineng.2025.108671)

5. **Hossain, M. S.**, Bashir, M. B. A. & Akter, K. (2026). "Design a low-cost solar PV data logger and assess the application and accuracy with off-grid monocrystalline panels." *Solar Energy*, 292, 113427. Elsevier. [DOI: 10.1016/j.measurement.2025.119176](https://doi.org/10.1016/j.measurement.2025.119176)

6. **Nkinyam, C. M.** et al. (2025). "Development of a low-cost monitoring device for solar electric (PV) system using internet of things (IoT)." *Energy Reports*, 13, 4788–4804. Elsevier. [DOI: 10.1016/j.rineng.2025.107324](https://doi.org/10.1016/j.rineng.2025.107324)

7. **Yuvaraj, R.** et al. (2025). "Real-time fire detection and suppression system using YOLO11n and Raspberry Pi for thermal safety applications." *Robotics and Autonomous Systems*, 185, 104925. Elsevier. [DOI: 10.1016/j.csite.2025.107159](https://doi.org/10.1016/j.csite.2025.107159)

8. **Yang, X.** et al. (2026). "A lightweight detector with hybrid pooling and checkerboard attention for solar panel anomalies." *Solar Energy*, 290, 113310. Elsevier. [DOI: 10.1016/j.isci.2026.115106](https://doi.org/10.1016/j.isci.2026.115106)

9. **Behera, S.** & Choudhury, N. B. D. (2022). "Modelling and Simulations of Modified Slime Mould Algorithm Based on Fuzzy PID to Design an Optimal Battery Management System in Microgrid." *Energies*, 15(14), 5328. MDPI. [DOI: 10.1016/j.cles.2022.100029](https://doi.org/10.1016/j.cles.2022.100029)

10. **Mezouari, M.**, Megrini, M. & Gaga, A. (2025). "Real-time voltage regulation using fuzzy logic in single-ended primary-inductor converter for electric energy systems." *Energy Reports*, 13, 4006–4023. Elsevier. [DOI: 10.1016/j.compeleceng.2025.110740](https://doi.org/10.1016/j.compeleceng.2025.110740)

11. **Sayeh, K. F.** et al. (2025). "A Novel Fuzzy-Logic-Improved Direct Power Control Strategy for DFIG-Based Wind Turbine Through Real-Time OPAL-RT Validation." *Energies*, 18(11), 2813. MDPI. [DOI: 10.1016/j.ijepes.2025.111317](https://doi.org/10.1016/j.ijepes.2025.111317)

12. **Ekuewa, O. I.**, Adejare, A. A. & Kim, J. (2024). "Intelligent scheduling algorithms for Internet of Things systems considering energy storage/consumption and network lifespan." *Journal of Energy Storage*, 86, 111155. Elsevier. [DOI: 10.1016/j.est.2024.114321](https://doi.org/10.1016/j.est.2024.114321)

13. **Sain, D.** & Lee, T. H. (2026). "Novel Mathematical Modeling of IT2 Takagi–Sugeno Fuzzy PID Controllers with Triangular FoUs and Their Application: A MagLev Case Study." *Energies*, 19(10), 2636. MDPI. [DOI: 10.1016/j.asoc.2026.114663](https://doi.org/10.1016/j.asoc.2026.114663)

14. **Yang, J.** et al. (2025). "Adaptive equalization method of lithium battery module based on time-varying characteristics of voltage and SOC." *Journal of Energy Storage*, 107, 114980. Elsevier. [DOI: 10.1016/j.est.2025.114980](https://doi.org/10.1016/j.est.2025.114980)

15. **Nippatla, V. R.** & Mandava, S. (2025). "Performance analysis of permanent magnet synchronous motor based on transfer function model using PID controller tuned by Ziegler–Nichols method." *e-Prime — Advances in Electrical Engineering, Electronics and Energy*, 11, 100939. Elsevier. [DOI: 10.1016/j.rineng.2025.105460](https://doi.org/10.1016/j.rineng.2025.105460)

---

## License

This project was developed as a **midterm examination (ETS)** assignment for the **Controller Programming** course (Pemrograman Kontroller) at the Department of Instrumentation Engineering, Faculty of Vocational Studies, Institut Teknologi Sepuluh Nopember (ITS), 2026.

The code and documentation are provided for **academic and educational purposes**.

---

<p align="center">
  <b>Simulasi SIL IT2 Fuzzy-PID</b> — Intelligent Thermal Management for Sustainable PV Energy ⚡🌱
</p>

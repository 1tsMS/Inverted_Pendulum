# 🎯 Inverted Pendulum on Cart (ESP32)

A real-world implementation of the classic inverted pendulum control problem using an ESP32, dual-loop PID control, and a belt-driven linear cart system.

This project demonstrates **nonlinear system stabilization**, **real-time control**, and **embedded implementation of cascade PID control**.

---

## ⚙️ System Overview

The system consists of:

* A **cart moving on a linear rail**
* A **free-rotating pendulum mounted on the cart**
* Real-time feedback using **encoders**
* A **cascade PID controller** to stabilize the pendulum

---

## 🧠 Control Architecture

This project uses a **cascade (dual-loop) PID control system**:

### 🔹 Inner Loop (Fast)

* Controls **pendulum angle**
* Responsible for **balancing**

### 🔹 Outer Loop (Slow)

* Controls **cart position**
* Keeps the cart near the center

```
Position PID → Target Angle → Angle PID → Motor Control
```

---

## 🧰 Hardware Components

### 🔌 Controller

* ESP32 WROOM Development Board

### ⚡ Motor & Drive

* 775 DC Motor (12V, 3000 RPM, dual shaft)
* BTS7960 High Power Motor Driver
* GT2 Belt Drive System
* 16 Teeth Pulley

### 🛤️ Mechanical System

* MGN9H Linear Guide Rail (0.5m)
* Sliding carriage (cart platform)

### 📡 Sensors

#### Pendulum Angle

* 600 PPR Incremental Rotary Encoder
* Mounted directly at the pendulum pivot

#### Cart Position

* AS5600 Magnetic Encoder (I2C)
* Magnet mounted on motor shaft

---

## ⚡ Power System

* 12V, 10A DC Power Supply

---

## 📐 Mechanical Design

* Pendulum:

  * Length: 25 cm
  * Material: Steel rod
  * No additional mass attached

* Drive system:

  * GT2 belt-driven linear motion
  * Compact and lightweight cart design

---

## 🧪 Features

* Real-time encoder feedback
* Multi-turn position tracking using AS5600
* Cascade PID control
* Software safety limits (angle + position)
* Serial-based live PID tuning
* Smooth motor control with filtering

---

## 🧠 Key Concepts Demonstrated

* Inverted pendulum stabilization
* Cascade control systems
* Real-time embedded control (ESP32)
* Encoder signal processing
* Nonlinear dynamics control

---

## 🚀 Getting Started

### 1. Hardware Setup

* Assemble cart on linear rail
* Mount pendulum securely on encoder shaft
* Install GT2 belt and pulley system
* Ensure smooth, low-friction motion

### 2. Wiring

* Connect motor driver to ESP32 GPIO pins
* Connect encoders (quadrature + I2C)
* Ensure **common ground across all components**


---

### 3. Upload Code

* Use Arduino IDE / PlatformIO
* Select ESP32 board
* Upload main control code

---

### 4. Serial Control

| Command | Function        |
| ------- | --------------- |
| S       | Start system    |
| X       | Stop system     |
| P       | Set angle Kp    |
| I       | Set angle Ki    |
| D       | Set angle Kd    |
| p       | Set position Kp |
| d       | Set position Kd |

---

## 🎯 PID Tuning Strategy

### Step 1: Tune Angle PID (Inner Loop)

* Disable position control
* Increase Kp until response is fast
* Add Kd to reduce oscillations
* Add small Ki for bias correction

### Step 2: Tune Position PID (Outer Loop)

* Enable position loop
* Start with small Kp
* Add Kd to reduce overshoot



---

## ⚠️ Safety Features

* Automatic motor cutoff if:

  * Pendulum angle exceeds safe limit
  * Cart exceeds track limits

---

## 🔧 Current Status

* Hardware: ✅ Completed
* Encoder feedback: ✅ Working
* PID control: ⚠️ In progress (balancing loop tuned)

---

## 🔮 Future Improvements

* LQR control implementation
* Higher control loop frequency
* Improved motor control using hardware PWM
* Better mechanical damping and rigidity
* Data logging and visualization

---

## 📸 Media

..

---

## 📌 Applications

* Control systems learning
* Robotics and automation education
* Embedded systems development
* Portfolio demonstration project

---

## 🧠 Author Notes

This project focuses on bridging **theory and practical implementation** of control systems. It highlights real-world challenges such as noise, delays, friction, and actuator limitations.

---

## 📜 License

MIT License


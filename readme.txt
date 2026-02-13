# Gesture Controlled Drone System  

> A real-time hand gesture based drone control interface using ESP32, MPU6050, and LDR with Python visualization and LiteWing drone integration.

---

## 📌 Overview

This project implements a gesture-based drone control system that converts hand movements into normalized flight commands (Roll, Pitch, Throttle) and transmits them wirelessly to a laptop for:

- 🎮 Real-time drone visualization  
- 🧠 Control signal processing  
- 🚀 Integration with a programmable LiteWing drone using Python  

The system works as a virtual RC transmitter powered by natural hand gestures.

---

## ✨ Features

- ✋ Gesture based Pitch & Roll control  
- 💡 LDR based Throttle control  
- 📡 Bluetooth / WiFi communication  
- 🎥 Real-time Python visualizer  
- 🚁 Programmable drone integration  
- 📊 Signal smoothing & stabilization  
- 🔐 Safety command limits  

---

## 🧠 System Architecture
Hand Gesture
↓
ESP32 (MPU6050 + LDR)
↓ Bluetooth / WiFi
Laptop (Python Control Layer)
↓
Visualizer / Drone

## HARDWARE NEEDED:

ESP 32
MPU6050 IMU
LDR SENSOR
10K RESISTOR

# GESTURE CONTROL MAP





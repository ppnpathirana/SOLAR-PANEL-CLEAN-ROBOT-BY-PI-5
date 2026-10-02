# 🤖 CleanBot v1.0 — Industrial Solar Panel Cleaning Robot

> An autonomous, AI-powered solar panel cleaning robot engineered for industrial applications. Built on the Raspberry Pi 5, CleanBot features zero-delay hardware safety systems, real-time computer vision, and a professional web-based command center.

![Python](https://img.shields.io/badge/Python-3.11-blue?logo=python)
![Flask](https://img.shields.io/badge/Flask-SocketIO-green?logo=flask)
![OpenCV](https://img.shields.io/badge/OpenCV-4.9-red?logo=opencv)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-purple)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red?logo=raspberrypi)

---

## 🌟 Executive Summary

**CleanBot** is designed to maintain peak efficiency for solar energy arrays through automated cleaning. By combining a custom-trained YOLOv8 computer vision model with robust edge-detection algorithms, the robot navigates safely and autonomously across solar panels. Its backend leverages a high-frequency, multi-threaded architecture to ensure instantaneous response to environmental hazards and emergency stop commands.

## 👥 Meet the Team

### 👨‍💻 Project Lead & Developer
**Pasindu**
* **Academic Profile:** Third Year Engineering Technology Student
* **Institution:** Sabaragamuwa University of Sri Lanka
* **Professional Affiliation:** IAENG Member (No. 566684)

### 👥 Contributors
**Sasidu Nisad**
* **Role:** Development & Technical Contributor
* **Bio:** A key contributor to this project, responsible for significant aspects of the development and implementation process. His contributions played an important role in bringing the project from development to completion.
* **LinkedIn:** [Sasidu Nisad](https://www.linkedin.com/in/sasindu-nisad-724388332/)

---

## 📸 Core Capabilities

- **Real-Time AI Vision** — Utilizes YOLOv8 for dirt and damage detection, overlaid with a multi-layered edge detection system (Canny, HSV, Sobel, and Contour).
- **Zero-Latency Motor Control** — Achieves a 200Hz movement execution loop using BTS7960 motor drivers for responsive and fluid navigation.
- **Industrial Safety Standards** — Features a hardware E-Stop and a 1ms polling watchdog thread running at `SCHED_FIFO` priority, guaranteeing immediate shutdown capabilities.
- **Custom Path Planning** — Includes an interactive dashboard designer to map custom cleaning routes, save them to a local SQLite database, and execute them autonomously.
- **Worldwide Telemetry Access** — Integrated with Tailscale VPN for secure, remote monitoring and control from any location.

---

## 🛠️ Hardware Specifications

| Component | Specification |
|-----------|--------------|
| **Controller** | Raspberry Pi 5 |
| **Drive System** | 2× DC Gear Motor (25GA-370, 12V, 280RPM with Encoder) |
| **Motor Drivers** | 2× BTS7960 (IBT-2) High-Current 43A Peak |
| **Cleaning Mechanism** | DC Gear Motor (25GA-370, 12V) via Relay |
| **Water System** | 12V Diaphragm Pump (TY-44520, 85PSI) |
| **Vision Sensor** | USB Camera (640×480 @ 30fps) |
| **Power Supply** | 12V 7Ah LiFePO4 Battery with BMS |

## 📌 GPIO Configuration

| Component | GPIO Pins |
|-----------|-----------|
| **Left Drive (BTS7960 #1)** | RPWM: 18, LPWM: 23, EN: 24 |
| **Right Drive (BTS7960 #2)**| RPWM: 19, LPWM: 20, EN: 21 |
| **Brush Relay** | GPIO 17 (Active-HIGH) |
| **Water Relay** | GPIO 26 (Active-HIGH) |
| **E-Stop Button** | GPIO 5 (Pull-Up) |

---

## ⚙️ Installation & Setup

### 1. Environment Preparation
Ensure your Raspberry Pi 5 is up to date and clone the repository:
```bash
git clone https://github.com/YOUR_USERNAME/cleanbot.git
cd cleanbot
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt --break-system-packages
```

### 3. AI Model Configuration
Place your custom-trained YOLOv8 model at the root directory:
```bash
/home/solarbot/cleanbot/best.pt
```
*(Note: If no model is provided, the system defaults to `yolov8n.pt` for basic object detection).*

### 4. Running the Application
**Manual Execution:**
```bash
python3 app.py
```
**Systemd Service (Auto-Start on Boot):**
```bash
sudo cp cleanbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cleanbot
sudo systemctl start cleanbot
```

---

## 🖥️ Command Center Dashboard

The CleanBot Dashboard is accessible locally via `http://<Pi-IP>:5000` or globally via Tailscale. 
It provides a professional, single-page interface equipped with:
- Live MJPEG video streaming with AI bounding box overlays.
- A top-down spatial map tracking the robot's physical position.
- Real-time telemetry charts (Chart.js) for Dirt Level, Cleaning Efficiency, and Speed.
- System health monitors (Battery, Water Level, WiFi Signal).

## 🛡️ Safety Architecture

CleanBot's architecture strictly isolates safety-critical operations from standard software loops:

1. **Hardware E-Stop:** Bypasses software states to trigger `_force_stop()` immediately.
2. **Watchdog Thread:** Dedicated thread polling every 1ms. Uses direct GPIO kills, avoiding Python's Global Interpreter Lock (GIL) overhead.
3. **Edge Protection:** Active during Auto-Clean mode. Analyzes four spatial zones simultaneously. (Note: Disabled during manual operation to allow operator flexibility).

---

## 📄 License

This project is licensed under the **MIT License**. You are encouraged to explore, modify, and build upon this architecture for your own industrial automation projects.

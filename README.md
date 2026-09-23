# LiBot - Autonomous Library Robot (Raspberry Pi Client)

![Python Version](https://img.shields.io/badge/python-3.8%2B-blue)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-cc2244)
![License](https://img.shields.io/badge/license-MIT-green)

## 📖 Table of Contents
- [About the Project](#-about-the-project)
- [System Architecture](#-system-architecture)
- [Supported Step-Types](#-supported-step-types)
- [Project Structure](#-project-structure)
- [Getting Started](#-getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage (How to Run)](#-usage-how-to-run)
- [Testing (Simulating Backend)](#-testing-simulating-backend)
- [Contributing](#-contributing)

## 📖 About the Project
**LiBot (Autonomous Library Robot for Book Search and Bookshelf Inventory Management)** is an intelligent robotics system designed to assist readers and librarians in a library environment[cite: 1]. 

This repository contains the **Client Application** running on the robot's **Raspberry Pi**. Instead of having hardcoded behaviors, the robot acts as a **Dynamic Workflow Engine**[cite: 2]. It maintains a TCP socket connection with a centralized **.NET Backend**[cite: 1]. The backend sends workflow configurations (JSON payloads) dynamically, and the robot executes them step-by-step, managing runtime variables and hardware interfaces autonomously.

## 🏗️ System Architecture
1. **TCP Communication Layer:** Listens to incoming workflow jobs from the .NET server[cite: 1].
2. **Workflow Engine:** Parses the JSON array of steps, manages state (Context), handles branching logic (IF/ELSE), and executes the tasks[cite: 2].
3. **Hardware Abstraction Layer (HAL):** Connects the software logic to physical components like ROS/Nav2 (Navigation), Cameras (Barcode/RFID scanning), Speakers (TTS), and Touch Displays.

## ⚙️ Supported Step-Types
The engine is built around standardized atomic step-types to ensure maximum flexibility[cite: 2]:
*   `MOVE`: Navigates the AMR to a specific 2D map coordinate or endpoint[cite: 2].
*   `SCAN`: Captures barcode or RFID data using hardware sensors[cite: 2].
*   `PROCESS_DATA`: Executes internal logic, data matching, or API calls[cite: 2].
*   `EVALUATE`: Checks runtime conditions for workflow branching[cite: 2].
*   `DISPLAY`: Updates the robot's interactive UI screen[cite: 2].
*   `WAIT`: Pauses execution waiting for hardware triggers or user touch events[cite: 2].
*   `NOTIFY`: Broadcasts voice announcements (TTS) or sends WebSocket alerts to librarians[cite: 2].

## 📁 Project Structure
```text
libot_pi/
├── main.py                     # Application entry point (Starts TCP Server)
├── config/                     
│   └── settings.json           # Configurations (IP, Port, Hardware config)
├── core/                       
│   ├── tcp_server.py           # TCP socket server implementation
│   ├── workflow_engine.py      # Parses and executes the JSON workflow
│   └── context.py              # Manages data passing between runtime steps
├── steps/                      # Implementations of atomic step-types
│   ├── move.py                 
│   ├── scan.py
│   ├── evaluate.py             
│   └── ...                 
├── hardware/                   # Hardware Abstraction Layer (HAL)
│   ├── navigation.py           # ROS / Motor controllers integration
│   ├── barcode_scanner.py      # Camera / Sensor controllers
│   └── ...               
└── requirements.txt            # Python dependencies

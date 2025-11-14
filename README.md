**# 🛸 ESP32 Drone QuadCopter**



A custom-designed \*\*ESP32-based drone flight controller\*\*, built from scratch to control a quadcopter using real-time motor control, sensor fusion, and ultra-low latency wireless communication.



This project includes both the \*\*Receiver (Flight Controller)\*\* and the \*\*Transmitter\*\* system using ESP-NOW.



---



**## 🚀 Project Overview**



The goal of this project is to develop a complete drone firmware stack on the ESP32 platform, enabling:



\- Stable quadcopter flight  

\- Real-time ESC + BLDC motor control  

\- Sensor fusion using IMU + barometer  

\- Low-latency wireless control  

\- LED indicators, buzzer alerts, and telemetry  



This repository will grow into a fully modular and expandable drone control system.



---



**## 🔧 High-Level Features**



\### ✈️ \*\*1. Flight Controller\*\*

\- 4× ESC outputs (PWM / OneShot)

\- Real-time motor updates  

\- PID-based stabilization  

\- Failsafe and arming logic  



\### 📡 \*\*2. Wireless Communication\*\*

\- ESP-NOW receiver  

\- Ultra-low latency control packets  

\- Custom packet structure  



\### 📊 \*\*3. Sensor Integration\*\*

\- \*\*MPU6050\*\* IMU (Gyro + Accelerometer)

\- \*\*BMP280\*\* Barometer (Altitude)

\- Sensor fusion for flight stabilization  



\### 🔔 \*\*4. Indicators \& Alerts\*\*

\- 8× LED strips  

\- Buzzer alerts (arming, battery, failsafe)



\### 🖥️ \*\*5. Telemetry\*\*

\- Real-time orientation  

\- Battery monitoring  

\- Status messages  



---



\## 🧭 System Architecture (High Level)



Transmitter ESP32 ─── ESP-NOW ───► Receiver ESP32

│

├─► MPU6050 (IMU)

├─► BMP280 (Barometer)

├─► ESC1/ESC2/ESC3/ESC4

├─► LED Strips

└─► Buzzer





Low latency, deterministic control is the primary focus.



---



\## 📁 Project Structure



ESP32\_Drone\_QuadCopter/

├── src/

│ ├── main.cpp

│ ├── espnow\_receiver.cpp

│ ├── pid\_controller.cpp

│ ├── motor\_control.cpp

│ ├── sensor\_mpu6050.cpp

│ └── sensor\_bmp280.cpp

├── include/

│ ├── pid\_controller.h

│ ├── motor\_control.h

│ └── sensor\_mpu6050.h

├── lib/

├── data/

├── README.md

└── .gitignore





(\*Files will be added as development progresses\*)



---



\## 🛠️ Hardware Used



\### \*\*Flight Controller\*\*

\- ESP32 Dev Module  

\- MPU6050 (IMU)  

\- BMP280 (barometer)  

\- 4× BLDC motors (1000KV)  

\- 4× ESCs  

\- LEDs + buzzer  



\### \*\*Transmitter\*\*

\- ESP32  

\- OLED Display  

\- 6 × Buttons  

\- Battery monitor  

\- Vibration motor  



---



\## 📅 Roadmap



\### \*\*Phase 1 — Base Setup\*\*

\- \[x] Git repo initialization  

\- \[ ] Project folder structure  

\- \[ ] ESP-NOW receiver  

\- \[ ] Telemetry output  



\### \*\*Phase 2 — Motor + Sensor Control\*\*

\- \[ ] ESC PWM control  

\- \[ ] MPU6050 basic readings  

\- \[ ] BMP280 integration  



\### \*\*Phase 3 — Flight Logic\*\*

\- \[ ] PID Stabilization  

\- \[ ] Arming sequence  

\- \[ ] Failsafe  



\### \*\*Phase 4 — Advanced Features\*\*

\- \[ ] LED animations  

\- \[ ] Buzzer alerts  

\- \[ ] 3D drone visualization (Web + Three.js)  

\- \[ ] Web dashboard  



---



\## 🤝 Contributions

PRs, suggestions, and improvements are welcome.  

This project is designed to be modular, educational, and open for community contribution.



---



\## 📄 License

To be added (MIT recommended).



---






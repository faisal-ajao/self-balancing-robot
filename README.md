# 🤖 Self-Balancing Robot with Arduino Nano, MPU6050 & PID Control

This project implements a **self-balancing two-wheeled robot** using the **Arduino Nano**, **MPU6050 IMU (Gyroscope + Accelerometer)**, and a **PID control algorithm**.  
The robot maintains balance in real time by continuously adjusting motor speeds based on orientation feedback.  

In robotics, maintaining stability in small robots on uneven surfaces is a classic challenge.  
This project demonstrates how **low-cost hardware + well-tuned control theory** can solve it efficiently.

---

## 🚀 Features
- Real-time balance control using **Arduino Nano + PID algorithm**.
- Dual PID loops: **Pitch control** for balance & **Yaw control** for rotation stability.
- **MPU6050 calibration utility** included for accurate motion sensing.
- **TT motors + L298N driver** for actuation.
- **Handmade chassis** built for testing in real-world uneven surfaces.
- Fritzing circuit layout (`.fzz` + `.png`) provided for easy hardware setup.

---

## 📦 Installation

Upload the provided Arduino sketches to your **Arduino Nano**:

```bash
# Clone the repository
git clone https://github.com/faisal-ajao/self-balancing-robot.git
cd self-balancing-robot

# Open in Arduino IDE (or PlatformIO) and upload:
# - calibration_mpu6050/calibration_mpu6050.ino  (for calibration)
# - self-balancing-robot.ino                     (main control loop)
```

---

## ▶️ Usage

1. **Calibrate MPU6050**  
   - Upload and run `calibration_mpu6050.ino`.  
   - Record the offsets printed in the Serial Monitor.  
   - These are automatically applied in `self-balancing-robot.ino`.  

2. **Run the Self-Balancing Robot**  
   - Upload `self-balancing-robot.ino` to the Arduino Nano.  
   - Place the robot upright and power it on.  
   - The PID controller will attempt to stabilize the robot in real time.  

---

## 📊 Output Example (Video)  
[![Watch the output](https://img.youtube.com/vi/FJ3LSyXKzSI/hqdefault.jpg)](https://youtu.be/FJ3LSyXKzSI?feature=shared)

---

## 📂 Project Structure
```
self-balancing-robot/
├── calibration_mpu6050/         # MPU6050 calibration sketch
│   └── calibration_mpu6050.ino
├── self-balancing-robot.ino     # Main self-balancing robot control code
├── setup_layout.fzz             # Fritzing circuit layout
├── setup_layout.png             # Circuit diagram (image)
└── README.md
```

---

## 🧠 Tech Stack
- **Arduino Nano** (main controller)
- **MPU6050** (I2C IMU sensor)
- **PID Control Algorithm**
- **L298N Motor Driver**
- **TT DC Motors**
- **Handmade chassis**
- **Fritzing** (for circuit diagrams)
- **Arduino C++**

### 📦 Arduino Library Dependencies
Make sure these libraries are installed via Arduino IDE Library Manager or manually:

- `I2Cdev` (I2C communication helpers)  
- `MPU6050_6Axis_MotionApps20` (DMP support for MPU6050)  
- `PID_v1` (PID control implementation)  

---

## 📚 Learnings & Applications
- Practical understanding of **PID control theory** and real-world tuning.  
- Insights into motor deadband compensation, sensor calibration, and stability.  
- Applicable foundations for more advanced robotics projects such as:
  - Autonomous navigation
  - Small-scale delivery robots
  - Balancing platforms on uneven terrain  

---

## 📜 License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

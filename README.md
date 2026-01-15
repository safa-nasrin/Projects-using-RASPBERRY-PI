# Human Following Robot using Raspberry Pi and YOLO

## Description
This project is a **human-following robot** powered by a Raspberry Pi 4 and YOLOv8 object detection.  
The robot detects a person using a webcam and follows them by controlling DC motors through an L298N motor driver.  

---

## Hardware Components
- Raspberry Pi 4 (2GB)
- L298N Motor Driver
- 2 DC Motors
- Li-ion Battery x 2
- Webcam
- Power Bank (to power Raspberry Pi & motors)
- Jumper Wires

---

## Software / Libraries
- Python 3.x
- OpenCV (`cv2`)
- YOLOv8 (`ultralytics` package)
- RPi.GPIO (GPIO control library for Raspberry Pi)
- Time (for delays)

---

## GPIO Connections (Raspberry Pi 4 → L298N → Motors)
| Raspberry Pi Pin | L298N / Function | Notes                    |
|-----------------|----------------|--------------------------|
| GPIO 17 (IN1)    | Motor driver IN1 | Controls left motor      |
| GPIO 18 (IN2)    | Motor driver IN2 | Controls left motor      |
| GPIO 22 (IN3)    | Motor driver IN3 | Controls right motor     |
| GPIO 23 (IN4)    | Motor driver IN4 | Controls right motor     |
| GPIO 5 (ENA)     | ENA (PWM)       | Controls speed left motor|
| GPIO 6 (ENB)     | ENB (PWM)       | Controls speed right motor|

---

## Robot Movements
- **Forward** → Moves both motors forward  
- **Left** → Turns left by controlling motor directions  
- **Right** → Turns right by controlling motor directions  
- **Stop** → Stops both motors  

---

## How to Run
1. Connect Raspberry Pi 4, L298N, motors, and webcam according to GPIO table  
2. Install required Python libraries:
```bash
pip install opencv-python ultralytics RPi.GPIO

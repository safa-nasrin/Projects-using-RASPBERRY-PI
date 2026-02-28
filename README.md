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
```

## Photos

![IMG_9556](https://github.com/user-attachments/assets/c4b4ec24-f9f7-4d9a-86bf-3a18f60c2a32)


## System Architecture Design

<img width="1024" height="1536" alt="Rasp" src="https://github.com/user-attachments/assets/76a65ebc-7b6d-40b0-bcfa-e263d7218cdc" />


## Human Following using Raspberry Pi 4 - 2GB

https://github.com/user-attachments/assets/81872c14-b564-4299-844a-7ab85323a140








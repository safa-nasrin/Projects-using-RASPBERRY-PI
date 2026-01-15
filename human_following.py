from ultralytics import YOLO
import cv2
import RPi.GPIO as GPIO
import time

# ---------------- GPIO Setup ----------------
IN1, IN2, IN3, IN4 = 17, 18, 22, 23
ENA, ENB = 5, 6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Setup PWM for speed control (30% duty cycle -> slow speed)
pwm_left = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(30)
pwm_right.start(30)

# ---------------- Movement Functions ----------------
def stop():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)

def forward():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)

def left():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 1)
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)

def right():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)

# ---------------- Load YOLO Model ----------------
model = YOLO("yolov8n.pt")

# Open Webcam
cap = cv2.VideoCapture(0)

print("🤖 Robot started. Press 'q' to quit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to capture frame")
            break

        # Run YOLO detection
        results = model(frame, imgsz=320, conf=0.5)

        person_box = None
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                if label == "person":  # Detect only humans
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx = int((x1 + x2) / 2)  # center x of person
                    cy = int((y1 + y2) / 2)  # center y of person
                    person_box = (cx, cy, x2 - x1, y2 - y1)
                    break

        if person_box:
            cx, cy, w, h = person_box
            frame_center = frame.shape[1] // 2  # middle of frame

            # Draw detection
            cv2.rectangle(frame, (cx - int(w/2), cy - int(h/2)), (cx + int(w/2), cy + int(h/2)), (0, 255, 0), 2)
            cv2.putText(frame, "Person", (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Control logic
            if abs(cx - frame_center) < 80:
                if w < 150:  # person is far
                    forward()
                    action = "Forward"
                else:  # person close enough
                    stop()
                    action = "Stop"
            elif cx < frame_center - 80:
                left()
                action = "Left"
            elif cx > frame_center + 80:
                right()
                action = "Right"
        else:
            stop()
            action = "No Person"

        # Show live camera feed
        cv2.putText(frame, f"Action: {action}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Live Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("✅ Camera released & GPIO cleaned up")
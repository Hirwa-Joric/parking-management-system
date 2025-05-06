import cv2
from ultralytics import YOLO
import pytesseract
import os
import time
import serial
import serial.tools.list_ports
import csv
from collections import Counter

# Load YOLOv8 model - Update path for Ubuntu
model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'weights', 'best.pt')
if not os.path.exists(model_path):
    print(f"[WARNING] Model not found at {model_path}. Please provide the correct path.")
    model_path = input("Enter the full path to your YOLO model: ")

model = YOLO(model_path)

# Plate save directory
save_dir = 'plates'
os.makedirs(save_dir, exist_ok=True)

# CSV log file
csv_file = 'plates_log.csv'
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Plate Number', 'Payment Status', 'Timestamp'])

# ===== Auto-detect Arduino Serial Port for Ubuntu =====
def detect_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        # On Ubuntu, Arduino typically connects as /dev/ttyACM0 or /dev/ttyUSB0
        if "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    return None

arduino_port = detect_arduino_port()
if arduino_port:
    print(f"[CONNECTED] Arduino on {arduino_port}")
    try:
        arduino = serial.Serial(arduino_port, 9600, timeout=1)
        time.sleep(2)
    except (serial.SerialException, OSError) as e:
        print(f"[ERROR] Failed to connect to Arduino: {e}")
        print("You may need to add your user to the 'dialout' group:")
        print("sudo usermod -a -G dialout $USER")
        print("Then log out and log back in.")
        arduino = None
else:
    print("[ERROR] Arduino not detected.")
    arduino = None

# ===== Ultrasonic Sensor Setup =====
import random
def mock_ultrasonic_distance():
    return random.choice([random.randint(10, 40)] + [random.randint(60, 150)] * 10)

# Initialize webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[ERROR] Failed to open webcam. Trying alternative device...")
    # Try alternative video devices on Ubuntu
    for i in range(1, 10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"[SUCCESS] Webcam opened on device {i}")
            break
    if not cap.isOpened():
        print("[FATAL ERROR] No webcam found. Check connection or permissions.")
        exit(1)

plate_buffer = []
entry_cooldown = 300  # 5 minutes
last_saved_plate = None
last_entry_time = 0

print("[SYSTEM] Ready. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to capture frame. Trying to reconnect...")
        cap.release()
        cap = cv2.VideoCapture(0)
        continue

    distance = mock_ultrasonic_distance()
    print(f"[SENSOR] Distance: {distance} cm")

    # Initialize results as None when distance > 50
    results = None
    
    if distance <= 50:
        results = model(frame)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Ensure coordinates are within frame boundaries
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                
                # Skip if resulting area is too small
                if x2 <= x1 or y2 <= y1:
                    continue
                    
                plate_img = frame[y1:y2, x1:x2]

                # Plate Image Processing
                gray = cv2.cvtColor(plate_img, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

                # OCR Extraction with Ubuntu path to Tesseract
                try:
                    plate_text = pytesseract.image_to_string(
                        thresh, config='--psm 8 --oem 3 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'
                    ).strip().replace(" ", "")
                except pytesseract.TesseractNotFoundError:
                    print("[ERROR] Tesseract not found. Install with: sudo apt install tesseract-ocr")
                    break

                # Plate Validation
                if "RA" in plate_text:
                    start_idx = plate_text.find("RA")
                    plate_candidate = plate_text[start_idx:]
                    if len(plate_candidate) >= 7:
                        plate_candidate = plate_candidate[:7]
                        prefix, digits, suffix = plate_candidate[:3], plate_candidate[3:6], plate_candidate[6]
                        if (prefix.isalpha() and prefix.isupper() and
                            digits.isdigit() and suffix.isalpha() and suffix.isupper()):
                            print(f"[VALID] Plate Detected: {plate_candidate}")
                            plate_buffer.append(plate_candidate)

                            # Decision after 3 captures
                            if len(plate_buffer) >= 3:
                                most_common = Counter(plate_buffer).most_common(1)[0][0]
                                current_time = time.time()

                                if (most_common != last_saved_plate or
                                    (current_time - last_entry_time) > entry_cooldown):

                                    with open(csv_file, 'a', newline='') as f:
                                        writer = csv.writer(f)
                                        writer.writerow([most_common, 0, time.strftime('%Y-%m-%d %H:%M:%S')])
                                    print(f"[SAVED] {most_common} logged to CSV.")

                                    if arduino:
                                        try:
                                            arduino.write(b'1')
                                            print("[GATE] Opening gate (sent '1')")
                                            time.sleep(15)  # Gate open duration
                                            arduino.write(b'0')
                                            print("[GATE] Closing gate (sent '0')")
                                        except serial.SerialException as e:
                                            print(f"[ERROR] Arduino communication failed: {e}")

                                    last_saved_plate = most_common
                                    last_entry_time = current_time
                                else:
                                    print("[SKIPPED] Duplicate within 5 min window.")

                                plate_buffer.clear()

                cv2.imshow("Plate", plate_img)
                cv2.imshow("Processed", thresh)
                time.sleep(0.5)

    try:
        annotated_frame = results[0].plot() if results else frame
    except (IndexError, TypeError):
        annotated_frame = frame

    cv2.imshow('Webcam Feed', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
if arduino:
    arduino.close()
cv2.destroyAllWindows()
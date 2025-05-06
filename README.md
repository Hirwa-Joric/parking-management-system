# Parking Management System

A computer vision-based parking management system that detects license plates and controls gate access.

## Features

- License plate detection using YOLOv8
- OCR for license plate recognition
- Arduino gate control integration
- Logging of license plates with timestamps

## Requirements

- Ubuntu Linux
- Webcam
- Arduino (optional for gate control)
- Python 3.8+

## Setup

1. Clone this repository
2. Run the setup script to install dependencies:
   ```
   cd parking-management-system
   ./setup.sh
   ```
3. Place your trained YOLOv8 model (`best.pt`) in the `weights` directory
   - If you don't have a model, the system will prompt you for the path on first run

## Hardware Setup

### Arduino Connection
- Connect the Arduino to a USB port
- The system will automatically detect it as `/dev/ttyACM0` or `/dev/ttyUSB0`
- Ensure your user has permissions to access the serial port (setup script adds you to dialout group)

### Camera Setup
- Connect a USB webcam to your computer
- The system will try to use the default camera (device 0) first
- If that fails, it will cycle through devices 1-9 to find a working camera

## Running the System

```
python3 car_entry.py
```

## Troubleshooting

### Serial Port Access
If you have permission issues with the Arduino port:
```
sudo usermod -a -G dialout $USER
```
Then log out and log back in.

### Camera Not Detected
If the webcam isn't detected, check:
```
ls -l /dev/video*
```
Verify your camera is connected and has the proper permissions.

### Tesseract OCR
If OCR isn't working:
```
sudo apt install tesseract-ocr
```

## License Plate Format

The system is configured to recognize license plates with format "RAXXX*" where:
- Prefix: 3 uppercase letters starting with "RA"
- Middle: 3 digits
- Suffix: 1 uppercase letter 
#!/bin/bash

echo "Setting up Parking Management System for Ubuntu..."

# Create weights directory if it doesn't exist
mkdir -p weights

# Install system dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y python3-pip python3-opencv tesseract-ocr

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install ultralytics pytesseract opencv-python serial

# Add current user to dialout group for serial port access
echo "Adding user to dialout group for Arduino access..."
sudo usermod -a -G dialout $USER
echo "NOTE: You may need to log out and log back in for the group change to take effect."

# Instructions for model
echo ""
echo "==== SETUP COMPLETE ===="
echo "You need to place your YOLOv8 model file (best.pt) in the 'weights' folder."
echo "If you don't have it, you'll be prompted to provide the path when running the program."
echo ""
echo "To run the program: python3 car_entry.py" 
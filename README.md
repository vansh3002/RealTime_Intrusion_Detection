# Real-Time Intrusion Detection System

## Overview
This project utilizes the DeepFace library and Arduino to detect and track unverified individuals in real-time. The system dynamically adjusts the position of a servo motor based on the detected intruder's movements, ensuring continuous tracking within the camera frame.

## Components
1. **DeepFace Library:** For face detection and verification against a database of pre-validated faces in the `Valid Faces` folder.
2. **Arduino and Servo Motor:** Controls servo movement to keep unverified individuals centered in the camera's frame.
3. **Valid Faces:** Contains images of authorized individuals who are recognized by the system.

## Algorithm Workflow
1. **Face Detection:** Captures real-time video feed and identifies faces.
2. **Face Verification:** Compares detected faces against the `Valid Faces` database.
3. **Servo Synchronization:** Adjusts servo motor position based on the face's coordinates to track intruders.
4. **Arduino Communication:** Sends commands from the computer to Arduino for servo control.

## Directory Structure
```
Realtime_Intrusion_Detection/
├── Aurdinocode/
|      ├─Aurdinocode.io        
├── Valid Faces/ # Database of verified individuals
├── RealTime_FaceDetection.py # Main script for face detection and servo control
├── RealTime_Face_Verification.py # Script focused on face verification
└── RealtimeDetection.pdf # Project documentation
```


## Setup Instructions
1. **Hardware:** Connect the camera and Arduino with the servo motor.
2. **Software:** Install Python, OpenCV, DeepFace, and other dependencies. Add images of authorized individuals to `Valid Faces`.
3. **Execution:** Run `RealTime_FaceDetection.py` to start the system.

## Conclusion
This system offers a real-time solution for intrusion detection and camera tracking by integrating face verification with dynamic servo control.

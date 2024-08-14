import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import time
import serial
from deepface import DeepFace

# Configure serial port
serial_port = 'COM7'  # Change this to your Arduino's serial port
baud_rate = 9600
ser = serial.Serial(serial_port, baud_rate, timeout=1)


def send_command(command):
    ser.write(command.encode())
    print("value sent")
    time.sleep(0.9)
    while True:
        try:
            response = ser.readline().decode().strip()  # Read response from Arduino
            print(response,12)
            if response == "OK":
                break  # Exit the loop when "OK" is received
        except:
            pass


cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

port = "COM7"
detector = FaceDetector()
servoPos = [90, 90] # initial servo position

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        #get the coordinate
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        # Extract the region of interest (ROI) from the image
        x=150
        top_left_x = int(fx - x)
        top_left_y = int(fy - x)

        # Calculate the coordinates of the bottom-right corner
        bottom_right_x = int(fx + x)
        bottom_right_y = int(fy + x)
        roi = img[top_left_y:bottom_right_y, top_left_x:bottom_right_x]
        # cv2.imwrite(output_path, roi)
        pos = [fx, fy]
        #convert coordinate to servo degree
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        if servoX < 0:
            servoX = 0
        elif servoX > 180:
            servoX = 180
        if servoY < 0:
            servoY = 0
        elif servoY > 180:
            servoY = 180

        servoPos[0] = servoX
        servoPos[1] = servoY
        x=150
        cv2.rectangle(img, (fx - x, fy - x), (fx + x, fy + x), (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )

    else:
        cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line


    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    msg = f"{180-servoPos[0]}:{servoPos[1]}"
    print(msg)
    send_command(msg)
    time.sleep(0.1)  # Add a delay of 50 milliseconds (0.05 seconds)

    cv2.imshow("Image", img)
    cv2.waitKey(1)

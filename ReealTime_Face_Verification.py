import cv2
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import time
import serial
from deepface import DeepFace
from deepface.basemodels import VGGFace
import pandas as pd


# models =VGGFace.load_model()
# # Configure serial port
serial_port = 'COM7'  # Change this to your Arduino's serial port
baud_rate = 9600
# ser = serial.Serial(serial_port, baud_rate, timeout=1)
# DeepFace.stream("C:\Extract face\Valid Faces")

#*******************************************************************************************************************************************

from deepface import DeepFace
import cv2

# Load your face database
database = {
    "Person1": "Load person1 image from the database",
    # Add more entries as needed
}

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

def draw_bounding_boxes(frame, result, servoPos):
    print(result)
    ws, hs = 1280, 720
    
    # Loop through each face detected
        # Extract face coordinates
    x, y, w, h = result["facial_area"]["x"], result["facial_area"]["y"], result["facial_area"]["w"], result["facial_area"]["h"]
    fx=(x+w/2)
    fy=(y+h/2)
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
    # Draw a rectangle around the detected face
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    cv2.putText(frame, "Not Verified", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    msg = f"{180-servoPos[0]}:{servoPos[1]}"
    # send_command(msg)
    time.sleep(0.1) 
    # cv2.imshow("Image", frame)
    cv2.waitKey(1)

# Function to recognise faces in the live camera feed
def recognise_faces_in_real_time():
    # Start capturing video from the default camera (0)
    cap = cv2.VideoCapture(0)
    port="COM7"
    servoPos = [90, 90]
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Failed to capture frame from camera. Exiting...")
            break        
        # Detect faces in the frame
        # print(type(frame))
        detected_faces = DeepFace.extract_faces(frame, detector_backend='opencv', enforce_detection = False)
        print((detected_faces[0]))
        # If faces are detectedf
        if len(detected_faces) > 0:
            # For each detected face
            for face in detected_faces:
                # Perform face recognition on the detected face
                result = DeepFace.find(face['face'], "C:\Extract face\Valid Faces", enforce_detection = False)
                
                # If the face is found in the database
                if result[0].shape[0]>0:
                    x, y, w, h = face["facial_area"]["x"], face["facial_area"]["y"], face["facial_area"]["w"], face["facial_area"]["h"]
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, "Verified", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    print("Face recognised")
                else:
                    draw_bounding_boxes(frame, face, servoPos)
                    # Face not found in the database, leave space for additional code
                    print("Detected face not found in the database. Provide space for additional code.")
                    
        
        # Display the live camera feed
        cv2.imshow('Live Feed', frame) 
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the camera and close the OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Run the function to recognise faces in real time
recognise_faces_in_real_time()


# def send_command(command):
#     ser.write(command.encode())
#     print("value sent")
#     time.sleep(0.9)
#     while True:
#         try:
#             response = ser.readline().decode().strip()  # Read response from Arduino
#             print(response,12)
#             if response == "OK":
#                 break  # Exit the loop when "OK" is received
#         except:
#             pass


# cap = cv2.VideoCapture(0)
# ws, hs = 1280, 720
# cap.set(3, ws)
# cap.set(4, hs)

# if not cap.isOpened():
#     print("Camera couldn't Access!!!")
#     exit()

# port = "COM7"
# # board = pyfirmata.Arduino(port)
# # servo_pinX = board.get_pin('d:9:s') #pin 9 Arduino
# # servo_pinY = board.get_pin('d:10:s') #pin 10 Arduino

# detector = FaceDetector()
# servoPos = [90, 90] # initial servo position

# while True:
#     success, img = cap.read()
#     img, bboxs = detector.findFaces(img, draw=False)

#     if bboxs:
#         #get the coordinate
#         fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
#         # Extract the region of interest (ROI) from the image
#         x=150
#         top_left_x = int(fx - x)
#         top_left_y = int(fy - x)

#         # Calculate the coordinates of the bottom-right corner
#         bottom_right_x = int(fx + x)
#         bottom_right_y = int(fy + x)
#         i=1
#         roi = img[top_left_y:bottom_right_y, top_left_x:bottom_right_x]
#         if roi is not None and not roi.empty():
#             cv2.imwrite(r"C:\Extract face\Extracted_images\\" + str(i) + ".jpg", roi)
#             df=DeepFace.find(img_path=r"C:\Extract face\Extracted_images\\" + str(i) + ".jpg",
#                             db_path="C:\Extract face\Valid Faces",
#                             model_name="VGG-Face",
#                             distance_metric="cosine",
#                             enforce_detection=False)
#             if(df[0].shape[0]==0):
#                 pos = [fx, fy]
#                 #convert coordinate to servo degree
#                 servoX = np.interp(fx, [0, ws], [0, 180])
#                 servoY = np.interp(fy, [0, hs], [0, 180])

#                 if servoX < 0:
#                     servoX = 0
#                 elif servoX > 180:
#                     servoX = 180
#                 if servoY < 0:
#                     servoY = 0
#                 elif servoY > 180:
#                     servoY = 180

#                 servoPos[0] = servoX
#                 servoPos[1] = servoY
#                 x=150
#                 cv2.rectangle(img, (fx - x, fy - x), (fx + x, fy + x), (0, 0, 255), 2)
#                 cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
#                 cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
#                 cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
#                 cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
#                 cv2.putText(img, "TARGET LOCKED", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )
#             else:
#                 cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
#                 cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
#                 cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
#                 cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
#                 cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line
#         else:
#             cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
#             cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
#             cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
#             cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
#             cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line                
#     else:
#         cv2.putText(img, "NO TARGET", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
#         cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
#         cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
#         cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
#         cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line


#     cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
#     cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

#     msg = f"{180-servoPos[0]}:{servoPos[1]}"
#     print(msg)
#     # send_command(msg)
#     time.sleep(0.1)  # Add a delay of 50 milliseconds (0.05 seconds)

#     cv2.imshow("Image", img)
#     cv2.waitKey(1)

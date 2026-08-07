import os
os.environ["GLOG_minloglevel"] = "2"
# Sets envioroment variable befoe MP import to help supress unncesseary logs

import time
import cv2 # Open CV
import mediapipe as mp
from mediapipe.tasks import python # New version of mp.solutions
from mediapipe.tasks.python import vision
# Other import stuff

MODEL_PATH = "hand_landmarker.task"
# The trained model file

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
# Shortening of variable names

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH), # Options Object, passes it as one of the settings
    running_mode=VisionRunningMode.VIDEO, # Set to Video for video stream
    num_hands=2 # Detect 2 hands
)
# Config object

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8), # Index finger
    (5, 9), (9, 10), (10, 11), (11, 12), # Middle finger
    (9, 13), (13, 14), (14, 15), (15, 16), # Ring finger
    (13, 17), (17, 18), (18, 19), (19, 20), # Pinky
    (0, 17), # Palm
]
# Hand landmark connections, used to draw lines between verticies

def draw_landmarks(frame, hand_landmarks_list, width, height):
    # hand_landmarks_list is list of hands detected, holds one entry per hand
    for hand_landmarks in hand_landmarks_list: # Does each hand individually
        # Convert normalized (0.0-1.0) coordinates to actual pixel coordinates
        points = [] # Storex pixel coordinates
        for landmark in hand_landmarks:
            x_px = int(landmark.x * width) 
            y_px = int(landmark.y * height) # Treated like a %
            points.append((x_px, y_px))
        # What this section does, is that it converts normalized 0-1 coordinates to pixel coordinates

        for connection in HAND_CONNECTIONS: # HAND_CONNECTIONS is the fixed list of index pairs from b4
            start_idx, end_idx = connection
            cv2.line(frame, points[start_idx], points[end_idx], (0, 255, 0), 2)
        # Draw connecting lines between joints

        for point in points:
            cv2.circle(frame, point, 4, (0, 0, 255), -1)
        # Draw a dot at each landmark

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # Creates a VideoCapture object
    # An object that knows how to communicate with the webcam

    if not cap.isOpened():
        print("Could not open camera.")
        return
    # Ensures that the webcame is opened correctly, informs the user if fails

    print("Camera opened. Press 'q' to quit.")

    start_time = time.time()
    # Used to compute real millisecond timestamps for each frame
    # MediaPipe uses it for motion and continuity between frames

    with HandLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break
            # Frame failsafe

            height, width, _ = frame.shape
            # Frame.shape for a color image is (height, width, channels)

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert BGR (OpenCV default) to RGB (MediaPipe expected)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame) # Wrap the raw numpy pixel array into MP's  image type

            frame_timestamp_ms = int((time.time() - start_time) * 1000)
            # Real elapsed time in milliseconds since start

            result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)
            # Run detection on this frame            

            if result.hand_landmarks: # Empty list if no hand was detected this frame
                draw_landmarks(frame, result.hand_landmarks, width, height)
            # Only draws if at least one hand is detected

            cv2.imshow("Hand Tracking", frame) # Display the frame in a seperate window

            if cv2.waitKey(1) & 0xFF == ord('q'): # Wait refreshes the window, waits 1 ms and sees if a key was pressed
                break
            # If q is pressed, break            

    cap.release() # Frees up webcam usage
    cv2.destroyAllWindows() # Closes opencv window

if __name__ == "__main__":
    main()
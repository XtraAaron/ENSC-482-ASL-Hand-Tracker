import os
os.environ["GLOG_minloglevel"] = "2"
# Sets envioroment variable befoe MP import to help supress unncesseary logs

import time
import struct
import socket
import cv2 # Open CV
import mediapipe as mp
from mediapipe.tasks import python # New version of mp.solutions
from mediapipe.tasks.python import vision
from OneEuroFilter import OneEuroFilter
# Other import stuff

MODEL_PATH = "hand_landmarker.task"
# The trained model file

# --- UDP setup for sending landmarks to Blender ---
UDP_IP = "127.0.0.1"  # localhost -- both programs run on this machine
UDP_PORT = 5052

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# One UDP socket, created once, reused every frame

NUM_FLOATS = 21 * 3
# 21 landmarks per hand, each with x, y, z -> 63 floats

# --- One Euro Filter setup ---
# One filter per (landmark, axis) -> 21 landmarks x 3 axes (x,y,z)
FILTER_CONFIG = {'freq': 30, 'mincutoff': 1.0, 'beta': 0.3, 'dcutoff': 1.0}
filters = [[OneEuroFilter(**FILTER_CONFIG) for _ in range(3)] for _ in range(21)]
# mincutoff -> lower = smoother when still, dcutoff -> filters the derivative estimate
# beta -> higher = less lag on fast motion
# Tune these while watching the Blender viewport if jitter/lag isn't right

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
# Shortening of variable names

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH), # Options Object, passes it as one of the settings
    running_mode=VisionRunningMode.VIDEO, # Set to Video for video stream
    num_hands = 1 # Detect 1 hand
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
    frame_count = 0
    with HandLandmarker.create_from_options(options) as landmarker: # Builds the detector object using the defined options
        while True:
            ret, frame = cap.read() # Grabs a frame from the camera
            # Ret is sucess
            # Frame is image data
            
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

                first_hand = result.hand_landmarks[0]
                # Take just the first detected hand for this first test

                frame_t = frame_timestamp_ms / 1000.0
                # One Euro Filter wants seconds, not ms

                flat_values = []
                for i, landmark in enumerate(first_hand):
                    fx = filters[i][0](landmark.x, frame_t)
                    fy = filters[i][1](landmark.y, frame_t)
                    fz = filters[i][2](landmark.z, frame_t)
                    flat_values.append(fx)
                    flat_values.append(fy)
                    flat_values.append(fz)
                
                frame_count += 1
                if frame_count % 30 == 0:
                    cmc1 = flat_values[1*3 : 1*3+3]
                    mcp2 = flat_values[2*3 : 2*3+3]
                    print(f"Frame {frame_timestamp_ms}ms")
                    print(f"  CMC1: {[round(v, 4) for v in cmc1]}")
                    print(f"  MCP2: {[round(v, 4) for v in mcp2]}")
                    print()
                # Debug stuff

                packet = struct.pack(f"{NUM_FLOATS}f", *flat_values)
                sock.sendto(packet, (UDP_IP, UDP_PORT))
                packet = struct.pack(f"{NUM_FLOATS}f", *flat_values)
                sock.sendto(packet, (UDP_IP, UDP_PORT))
                # Pack as raw binary floats and send to Blender over UDP

            cv2.imshow("Hand Tracking", frame) # Display the frame in a seperate window

            if cv2.waitKey(1) & 0xFF == ord('q'): # Wait refreshes the window, waits 1 ms and sees if a key was pressed
                break
            # If q is pressed, break            

    cap.release() # Frees up webcam usage
    cv2.destroyAllWindows() # Closes opencv window

if __name__ == "__main__":
    main()
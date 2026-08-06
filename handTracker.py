import os
os.environ["GLOG_minloglevel"] = "2"

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Path to the model file you downloaded. Update this if it's not in the same folder.
MODEL_PATH = "hand_landmarker.task"

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Set up the landmarker in VIDEO mode since we're feeding it a live stream frame by frame
options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=2
)

# 21 hand landmark connections, used to draw the skeleton lines between points
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),       # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),       # Index finger
    (5, 9), (9, 10), (10, 11), (11, 12), # Middle finger
    (9, 13), (13, 14), (14, 15), (15, 16), # Ring finger
    (13, 17), (17, 18), (18, 19), (19, 20), # Pinky
    (0, 17),                              # Palm
]

def draw_landmarks(frame, hand_landmarks_list, width, height):
    for hand_landmarks in hand_landmarks_list:
        # Convert normalized (0.0-1.0) coordinates to actual pixel coordinates
        points = []
        for landmark in hand_landmarks:
            x_px = int(landmark.x * width)
            y_px = int(landmark.y * height)
            points.append((x_px, y_px))

        # Draw connecting lines between joints
        for connection in HAND_CONNECTIONS:
            start_idx, end_idx = connection
            cv2.line(frame, points[start_idx], points[end_idx], (0, 255, 0), 2)

        # Draw a dot at each landmark
        for point in points:
            cv2.circle(frame, point, 4, (0, 0, 255), -1)

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Could not open camera.")
        return

    print("Camera opened. Press 'q' to quit.")

    with HandLandmarker.create_from_options(options) as landmarker:
        frame_timestamp_ms = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            height, width, _ = frame.shape

            # Convert BGR (OpenCV default) to RGB (MediaPipe expects this)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # Run detection on this frame
            result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)
            frame_timestamp_ms += 1

            if result.hand_landmarks:
                draw_landmarks(frame, result.hand_landmarks, width, height)

            cv2.imshow("Hand Tracking", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
import cv2 # Import OpenCV library

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # Creates a VideoCapture object
# An object that knows how to communicate with the webcam

if not cap.isOpened():
    print("Could not open camera.")
    exit() # Ensures that the webcame is opened correctly

print("Camera opened. Press 'q' to quit.")

while True:
    ret, frame = cap.read() # Takes a picture, if sucess ret is a boolian so true of false
    # Frame is the image that has been taken
    if not ret:
        print("Failed to grab frame.")
        break
    # Error handling for failed frame data

    cv2.imshow("Camera Test", frame) # Display the frame in a seperate window

    if cv2.waitKey(1) & 0xFF == ord('q'): # Wait refreshes the window, waits 1 ms and sees if a key was pressed
        break
    # If q is pressed, break

cap.release() # Frees up webcam usage
cv2.destroyAllWindows() # Closes opencv window
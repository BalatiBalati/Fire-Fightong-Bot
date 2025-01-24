import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

Fire_Reported = 0

# Continuous capture
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Grab the raw NumPy array representing the image
    image = frame.array

    # Resize the image to a manageable size
    image = cv2.resize(image, (1000, 600))

    # Apply Gaussian blur to the image
    blur = cv2.GaussianBlur(image, (15, 15), 0)

    # Convert the image to HSV (Hue, Saturation, Value) color space
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Define the lower and upper boundaries for detecting fire (or flames)
    lower = np.array([18, 50, 50], dtype='uint8')
    upper = np.array([35, 255, 255], dtype='uint8')

    # Create a mask to extract regions of interest (ROI) based on the color range
    mask = cv2.inRange(hsv, lower, upper)

    # Apply the mask to the original image
    output = cv2.bitwise_and(image, image, mask=mask)

    # Count the number of non-zero pixels (regions with fire color)
    number_of_total = cv2.countNonZero(mask)

    # If the number of fire-colored pixels exceeds a threshold, detect fire
    if int(number_of_total) > 15000:  # Threshold for fire detection
        print('Fire is Detected')
        Fire_Reported = 1
    else:
        Fire_Reported = 0

    # Show the output frame
    cv2.imshow("Output", output)

    # Check if the user pressed the 'q' key to break the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

# Release the camera resources and close the OpenCV window
cv2.destroyAllWindows()
camera.close()

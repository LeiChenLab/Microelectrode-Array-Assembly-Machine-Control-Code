import cv2
import numpy as np

def nothing(x):
    pass

# Create a window to display the image
cv2.namedWindow("HSV Calibration")

# Create trackbars for adjusting HSV range
cv2.createTrackbar("Lower Hue", "HSV Calibration", 0, 179, nothing)
cv2.createTrackbar("Lower Sat", "HSV Calibration", 0, 255, nothing)
cv2.createTrackbar("Lower Val", "HSV Calibration", 0, 255, nothing)
cv2.createTrackbar("Upper Hue", "HSV Calibration", 179, 179, nothing)
cv2.createTrackbar("Upper Sat", "HSV Calibration", 255, 255, nothing)
cv2.createTrackbar("Upper Val", "HSV Calibration", 255, 255, nothing)

# Capture video from camera
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Get the current positions of all trackbars
    lower_hue = cv2.getTrackbarPos("Lower Hue", "HSV Calibration")
    lower_sat = cv2.getTrackbarPos("Lower Sat", "HSV Calibration")
    lower_val = cv2.getTrackbarPos("Lower Val", "HSV Calibration")
    upper_hue = cv2.getTrackbarPos("Upper Hue", "HSV Calibration")
    upper_sat = cv2.getTrackbarPos("Upper Sat", "HSV Calibration")
    upper_val = cv2.getTrackbarPos("Upper Val", "HSV Calibration")
    
    # Define lower and upper bounds for color detection
    lower_bound = np.array([lower_hue, lower_sat, lower_val])
    upper_bound = np.array([upper_hue, upper_sat, upper_val])
    
    # Create a mask for the selected color range
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Show the original frame and mask side by side
    cv2.imshow("Original Frame", frame)
    cv2.imshow("Mask", mask)
    
    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources and close windows
cap.release()
cv2.destroyAllWindows()

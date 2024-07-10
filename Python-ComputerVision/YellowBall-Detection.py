import cv2 as cv
import numpy as np
from time import sleep

# '***' stands for things to modify for your own webcam, display, and ball if needed

# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
    cap = cv.VideoCapture(0)
    # CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Set the image resolution to 480x480 *** Note increasing resolution increases processing power used, and may slow down video feed
        frame = cv.resize(frame, (480, 480))

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

        # Define the range of yellow color in HSV [Hue, Saturation, Value] ***
        # You can get these values via the method explained in the tutorial. You can also change the variable names to suit your color ball
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        mask = cv.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10:  # Only consider large enough objects *** If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
                # Draw a circle around the yellow ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a dot in the center of the yellow ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv.destroyAllWindows()

# Call the function to detect the yellow ball
detect_yellow_ball()

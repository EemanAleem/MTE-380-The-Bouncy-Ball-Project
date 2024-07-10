import cv2
import numpy as np
from time import sleep

# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam
    cap = cv2.VideoCapture(0)
    # The first parameters 3 and 4 describe the width and height of 600 pixels
    cap.set(3, 600)
    cap.set(4, 600)
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        #crop a 200x200 region from the center of the frame (pixels 200-400 in the X/Y)
        frame = frame[200:400, 200:400]

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

        # Define the range of yellow color in HSV [Hue, Saturation, Value]
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        # Pixels in the range are set to white (255) and those that aren't are set to black (0)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv2.contour Area function
            largest_contour = max(contours, key=cv2.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if radius > 10:  # Only consider large enough objects
                # Draw a dot in the center of the yellow ball
                cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Delay 0.5 seconds
        sleep(0.5)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv2.destroyAllWindows()

# Call the function to detect the yellow ball
detect_yellow_ball()

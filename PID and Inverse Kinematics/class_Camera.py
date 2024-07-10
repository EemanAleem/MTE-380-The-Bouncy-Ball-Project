from picamera2 import Picamera2
import cv2
import numpy as np
import threading

lock = threading.Lock()

class Camera:
    def __init__(self):
        # Initialize and configure the camera
        self.picam2 = Picamera2()
        self.height = 480
        self.width = 480
        self.config = self.picam2.create_video_configuration(
            main={"format": 'XRGB8888', "size": (self.height, self.width)},
            controls={
                "FrameDurationLimits": (8333, 8333),
                "ExposureTime": 8000
            }
        )
        self.picam2.configure(self.config)

        # Define the HSV range for fluorescent pink
        self.lower_pink = np.array([140, 150, 50])  # H: around 140 degrees
        self.upper_pink = np.array([180, 255, 255])  # H: up to around 170 degrees

        # Start the camera
        self.picam2.start()

    def take_pic(self):
        # Capture an image
        image = self.picam2.capture_array()
        return image

    def show_video(self, image):
        # Display live video
        cv2.imshow("Live", image)
        cv2.waitKey(1)

    def find_ball(self, image):
        # Convert image to HSV color space
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask based on color range
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            area = cv2.contourArea(largest_contour)  # Calculate area

            if area > 200:  # Threshold to ignore noise
                # Draw circle on image
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                self.show_video(image)

                # Calculate distance and height
                d = radius * 2
                h = 10000 / d

                # Adjust center coordinates
                x -= self.height / 2
                y -= self.width / 2
                x, y = -y, x

                return int(x), int(y), int(area)  # Return image coordinates and area

        self.show_video(image)
        return -1, -1, 0  # Return if no ball detected

    def clean_up_cam(self):
        # Stop and close the camera
        self.picam2.stop()
        self.picam2.close()

        # Close all OpenCV windows
        cv2.destroyAllWindows()

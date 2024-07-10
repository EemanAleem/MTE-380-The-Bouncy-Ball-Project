import class_BBRobot
import class_Camera
import class_PID
import time
import threading
import numpy as np

# Image size (height, width) and number of channels (here 3 channels = RGB)
height = 480
width = 640
channels = 3

# Create an empty image (all pixels initialized to 0)
image = np.zeros((height, width, channels), dtype=np.uint8)

# Servo numbers used for the robot
ids = [1, 2, 3]

# Coefficients for PID and coefficient to determine the size of phi
K_PID = [0.015, 0.0001, 0.0051]  # 0.015, 0.0001, 0.0051
k = 1
a = 1

# Instantiate robot, camera, and PID controller
Robot = class_BBRobot.BBrobot(ids)
camera = class_Camera.Camera()
pid = class_PID.PID(K_PID, k, a)

# Prepare the robot and move it to the initial position
Robot.set_up()
Robot.Initialize_posture()
pz_ini = Robot.ini_pos[2]

frame_count = 0
start_time = time.time()
img_start_time = time.time()
rob_start_time = time.time()
fps = 0  # Initialize to 0
img_fps = 0
rob_fps = 0

# Ball coordinates
x = -1
y = -1
area = -1
goal = [100, 0]

def get_img():
    global image, img_fps, img_start_time
    img_frame_count = 0
    while True:
        image = camera.take_pic()
        # Calculate img_fps
        img_frame_count += 1
        if img_frame_count == 100:
            img_end_time = time.time()
            img_elapsed_time = img_end_time - img_start_time
            img_fps = 100 / img_elapsed_time
            img_start_time = img_end_time
            img_frame_count = 0

def cont_rob():
    global x, y, area, rob_fps, rob_start_time
    rob_frame_count = 0
    while True:
        x, y, area = camera.find_ball(image)
        # Calculate rob_fps
        rob_frame_count += 1
        if rob_frame_count == 100:
            rob_end_time = time.time()
            rob_elapsed_time = rob_end_time - rob_start_time
            rob_fps = 100 / rob_elapsed_time
            rob_start_time = rob_end_time
            rob_frame_count = 0

try:
    camera_thread = threading.Thread(target=get_img)
    rob_thread = threading.Thread(target=cont_rob)
    camera_thread.start()
    rob_thread.start()

    while True:
        Current_value = [x, y, area]
        if x != -1:
            theta, phi = pid.compute(goal, Current_value)
            pos = [theta, phi, pz_ini]
            Robot.control_t_posture(pos, 0.01)
        print(f"img_fps: {img_fps}, rob_fps: {rob_fps}")

finally:
    Robot.clean_up()
    camera.clean_up_cam()

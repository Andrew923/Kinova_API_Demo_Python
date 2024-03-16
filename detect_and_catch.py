"""
This file combines the functionality of creating a connection, getting video from the robot and getting it to catch a ball
"""
from connect import RobotAPI
from vision import BallDetector


# Parameters
ip = "192.168.2.10"
port = 10000 # Default TCP port
credentials = ("admin", "admin") # TODO: Change to your own username-password

# Start connection
robot_api = RobotAPI(ip, port, credentials)
robot_api.create_connection()

# Create video processing entity. This will localize the ball in the space
# Attach the vision object to the same session and compute the intrinsic matrix from device info
vision_process = BallDetector(robot_api)
while True:
    global_point = vision_process.find_object()
    print(global_point)
# Close the connection
# TODO: Remove endless loop
robot_api.close_connection()
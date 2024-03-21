"""
This file combines the functionality of creating a connection, getting video from the robot and getting it to catch a ball
"""
from connect import RobotConnect
from vision import BallDetector
from arm_mover import ArmMover
import cv2


# Parameters
ip = "192.168.2.10"
port = 10000 # Default TCP port
credentials = ("admin", "admin") # TODO: Change to your own username-password

# Start connection
robot_connection = RobotConnect(ip, port, credentials)
robot_connection.create_connection()

# Attach the vision and movement objects to the same session
vision_process = BallDetector(robot_connection)
mover = ArmMover(robot_connection)

# Move the arm to its object tracking position
track_status = mover.object_tracking_position() # Blocking



# When the user presses enter, have the robot grasp the ball
while not (cv2.waitKey(33) == ord('a')):
    vision_process.find_object()

# Once user is satisfied with tracking, get the robot to catch the ball
mover.catch_target(vision_process.global_point)

# End the program
vision_process.end_vision()
robot_connection.close_connection()

"""
Enhanced ArmMover with dirty flag system for efficient command sending
"""

import numpy as np
import threading
import time
from kortex_api.autogen.messages import Base_pb2, Common_pb2

class ArmMover:
    def __init__(self, robot_connection):
        self.robot_connection = robot_connection

        # Gripper info
        self.gripper_request = Base_pb2.GripperRequest()
        self.gripper_request.mode = Base_pb2.GRIPPER_POSITION
        if not self.robot_connection.debug:
            self.gripper_measure = self.robot_connection.base.GetMeasuredGripperMovement(self.gripper_request)
        else:
            self.gripper_measure = None
        
        # Velocity control parameters
        self.linear_velocity = 1  # m/s
        self.angular_velocity = 30.0  # deg/s
        self.gripper_speed = 0.4
        
        # Current commands (initialize all to zero)
        self.current_twist = Base_pb2.TwistCommand()
        self.current_twist.reference_frame = Common_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        
        self.current_joint_speeds = Base_pb2.JointSpeeds()
        self.current_gripper_speed = 0.0
        
        # Dirty flags
        self.twist_dirty = False
        self.joint_speeds_dirty = False
        self.gripper_dirty = False
        
        # Thread control
        self.running = True
        self.command_thread = threading.Thread(target=self._send_continuous_commands)
        self.command_thread.start()

    def get_gripper_info(self):
        return self.gripper_measure

    def _send_continuous_commands(self):
        """Thread that efficiently sends only changed commands"""
        while self.running:
            try:
                # Only send commands that have changed
                if self.twist_dirty:
                    if not self.robot_connection.debug:
                        self.robot_connection.base.SendTwistCommand(self.current_twist)
                    else:
                        print(self.current_twist)
                    self.twist_dirty = False
                
                if self.joint_speeds_dirty:
                    if not self.robot_connection.debug:
                        self.robot_connection.base.SendJointSpeedsCommand(self.current_joint_speeds)
                    else:
                        print(self.current_joint_speeds)
                    self.joint_speeds_dirty = False
                
                if self.gripper_dirty:
                    gripper_command = Base_pb2.GripperCommand()
                    finger = gripper_command.gripper.finger.add()
                    gripper_command.mode = Base_pb2.GRIPPER_SPEED
                    finger.value = self.current_gripper_speed

                    if not self.robot_connection.debug:
                        self.gripper_request = Base_pb2.GripperRequest()
                        self.gripper_request.mode = Base_pb2.GRIPPER_POSITION
                        self.gripper_measure = self.robot_connection.base.GetMeasuredGripperMovement(self.gripper_request)

                        self.robot_connection.base.SendGripperCommand(gripper_command)
                    else:
                        print(gripper_command)
                    self.gripper_dirty = False
                
                time.sleep(0.02)  # 50Hz command rate
                
            except Exception as e:
                print(f"Error sending commands: {e}")
                if not self.running:  # If we're shutting down, break the loop
                    break
                time.sleep(0.1)  # Brief pause before retrying

    def stop(self):
        """Stop all movement and clean up"""
        self.running = False
        
        # Set all commands to zero and mark dirty
        self.set_cartesian_velocity()
        self.set_joint_velocities([0]*6)  # Assuming 6DOF arm
        self.move_gripper(0)
        
        # Wait for thread to finish
        self.command_thread.join()
        
        # Send final stop command
        self.robot_connection.base.Stop()
        
    def set_cartesian_velocity(self, linear_x=0, linear_y=0, linear_z=0,
                            angular_x=0, angular_y=0, angular_z=0):
        print(f"Setting cartesian velocity: {linear_x}, {linear_y}, {linear_z}, "
              f"{angular_x}, {angular_y}, {angular_z}")
        """Set cartesian velocity command and mark as dirty"""
        new_linear_x = linear_x * self.linear_velocity
        new_linear_y = linear_y * self.linear_velocity
        new_linear_z = linear_z * self.linear_velocity
        new_angular_x = angular_x * self.angular_velocity
        new_angular_y = angular_y * self.angular_velocity
        new_angular_z = angular_z * self.angular_velocity
        
        # Only update and mark dirty if values actually changed
        if (self.current_twist.twist.linear_x != new_linear_x or
            self.current_twist.twist.linear_y != new_linear_y or
            self.current_twist.twist.linear_z != new_linear_z or
            self.current_twist.twist.angular_x != new_angular_x or
            self.current_twist.twist.angular_y != new_angular_y or
            self.current_twist.twist.angular_z != new_angular_z):
            
            self.current_twist.twist.linear_x = new_linear_x
            self.current_twist.twist.linear_y = new_linear_y
            self.current_twist.twist.linear_z = new_linear_z
            self.current_twist.twist.angular_x = new_angular_x
            self.current_twist.twist.angular_y = new_angular_y
            self.current_twist.twist.angular_z = new_angular_z
            self.twist_dirty = True

    def set_joint_velocities(self, joint_velocities):
        """Set joint velocity command and mark as dirty"""
        # Create new JointSpeeds message
        new_joint_speeds = Base_pb2.JointSpeeds()
        for i, vel in enumerate(joint_velocities):
            js = new_joint_speeds.joint_speeds.add()
            js.joint_identifier = i
            js.value = vel
            js.duration = 0
        
        # Only update if the new command is different
        if ((len(self.current_joint_speeds.joint_speeds) != len(new_joint_speeds.joint_speeds))
             or any(old.value != new.value for old, new in 
                zip(self.current_joint_speeds.joint_speeds, new_joint_speeds.joint_speeds))):
            
            self.current_joint_speeds = new_joint_speeds
            self.joint_speeds_dirty = True

    def move_gripper(self, direction):
        """Move gripper at constant speed and mark as dirty"""
        new_speed = direction * self.gripper_speed
        if self.current_gripper_speed != new_speed:
            self.current_gripper_speed = new_speed
            self.gripper_dirty = True

    def stop_gripper(self):
        """Stop gripper movement"""
        self.current_gripper_speed = 0.0

    def object_tracking_position(self):
        """Move to predefined tracking position (using position control)"""
        track_action = Base_pb2.Action()
        track_action.name = "Position to track the target"
        cartesian_pose = track_action.reach_pose.target_pose
        cartesian_pose.x = 0.38
        cartesian_pose.y = 0.00
        cartesian_pose.z = 0.34
        cartesian_pose.theta_x = 180
        cartesian_pose.theta_y = 0
        cartesian_pose.theta_z = 90
        return self._execute_movement(track_action)

    def _execute_movement(self, action):
        """Execute a position-based movement (for initial positioning)"""
        if self.robot_connection.debug:
            print(action)
            return True
        e = threading.Event()
        notification_handle = self.robot_connection.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        self.robot_connection.base.ExecuteAction(action)
        finished = e.wait(20)
        self.robot_connection.base.Unsubscribe(notification_handle)
        return finished

    def _check_for_end_or_abort(self, e):
        def check(notification, e=e):
            if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                e.set()
        return check

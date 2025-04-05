from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import GripperCyclic_pb2
import csv
import os
import time

from kortex_api.autogen.messages import BaseCyclic_pb2

class DataCollector:

    def __init__(self, base, name):
        self.base = base            # feedback base
        # self.base_cyclic = base_cyclic
        self.base_command = BaseCyclic_pb2.Command()
        # self.base_command = BaseCyclic_pb2.Command()    # this is for refreshing feedback
        # self.gripper = gripper      # feedback gripper
        self.subject = name         # name of person's new folder to store stuff in
        self.filename = "Test 1"      # csv file name

        self.headers =  ["Time", "Gripper Pos",
             "Joint 0", "Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5",
             "X-Pos", "Y-Pos", "Z-Pos", "X-Theta", "Y-Theta", "Z-Theta"]
        
        self.test_number = 0
        self.start_time = 0

        self.filepath = os.path.join(self.subject, self.filename)
        os.makedirs(self.subject, exist_ok=True)

        with open(self.filepath, mode="w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(self.headers)
    
    def start_test(self, test_number):
        self.test_number = test_number
        self.start_time = time.time()



    def add_row(self, gripper_info):
        # base_feedback = self.base_cyclic.Refresh(self.base_command)
        # print(base_feedback.interconnect.gripper_feedback.moto)

        # self.base.GetMeasuredGripperMovement(gripper_request)
        
        # print("Printing end effector calculations:")
        end_effector = self.calc_end()
        # print(end_effector)
        # print("Printing joints:")
        joints = self.get_joints()
        # print(joints)
        # print("Printing gripper:")
        # print(gripper_info)

        # print()

        # print(joints.joint_angles)
        # print(joints.joint_angles[0])
        # print(joints.joint_angles[0].value)

        # print(gripper_info.finger)
        # print(gripper_info.finger[0])
        # print(gripper_info.finger[0].value)

        # print(end_effector.x)
        
        # print("should error after")
        
        # time.sleep(1)
        
        row = {
            "Time": time.time(),
            "Gripper Pos": gripper_info.finger[0].value,
            "Joint 0": joints.joint_angles[0].value,
            "Joint 1": joints.joint_angles[1].value,
            "Joint 2": joints.joint_angles[2].value,
            "Joint 3": joints.joint_angles[3].value,
            "Joint 4": joints.joint_angles[4].value,
            "Joint 5": joints.joint_angles[5].value,
            "X-Pos": end_effector.x,
            "Y-Pos": end_effector.y,
            "Z-Pos": end_effector.z,
            "X-Theta": end_effector.theta_x,
            "Y-Theta": end_effector.theta_y,
            "Z-Theta": end_effector.theta_z,
        }
        # print(row)
        # print(row.keys())

        # self.filepath = os.path.join(self.subject, self.filename)
        with open(self.filepath, mode="a", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=row.keys())
            writer.writerow(row)

    # def get_gripper(self):

    #     feedback_request = GripperCyclic_pb2.FeedbackRequest()
    #     print(feedback_request)
    #     feedback_request.message_id = 1
    #     try:
    #         feedback = self.gripper.RefreshFeedback(feedback_request)
    #         vals = [feedback.motor[0].position,
    #                 feedback.motor[0].current,
    #                 feedback.motor[0].temperature]
    #     except KServerException as ex:
    #         print("Unable to get gripper data")
    #         print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
    #         print("Caught expected error: {}".format(ex))
    #         return None
        
    #     return vals

    def calc_end(self):
        # Current arm's joint angles (in home position)
        try:
            # print("Getting Angles for every joint...")
            input_joint_angles = self.base.GetMeasuredJointAngles()
        except KServerException as ex:
            print("Unable to get joint angles")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return None

        # print("Joint ID : Joint Angle")
        # for joint_angle in input_joint_angles.joint_angles:
        #     print(joint_angle.joint_identifier, " : ", joint_angle.value)
        # print()
        
        # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
        try:
            # print("Computing Foward Kinematics using joint angles...")
            pose = self.base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to compute forward kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return None

        # print("Pose calculated : ")
        # print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
        # print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
        # print()

        return pose

    def get_joints(self):
        # get robot's pose (by using forward kinematics)
        try:
            input_joint_angles = self.base.GetMeasuredJointAngles()
            # pose = base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to get current robot pose")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        return input_joint_angles

        # # Object containing cartesian coordinates and Angle Guess
        # input_IkData = Base_pb2.IKData()
        
        # # Fill the IKData Object with the cartesian coordinates that need to be converted
        # input_IkData.cartesian_pose.x = pose.x
        # input_IkData.cartesian_pose.y = pose.y
        # input_IkData.cartesian_pose.z = pose.z
        # input_IkData.cartesian_pose.theta_x = pose.theta_x
        # input_IkData.cartesian_pose.theta_y = pose.theta_y
        # input_IkData.cartesian_pose.theta_z = pose.theta_z

        # # Fill the IKData Object with the guessed joint angles
        # for joint_angle in input_joint_angles.joint_angles :
        #     jAngle = input_IkData.guess.joint_angles.add()
        #     # '- 1' to generate an actual "guess" for current joint angles
        #     jAngle.value = joint_angle.value - 1
        
        # try:
        #     # print("Computing Inverse Kinematics using joint angles and pose...")
        #     computed_joint_angles = base.ComputeInverseKinematics(input_IkData)
        # except KServerException as ex:
        #     print("Unable to compute inverse kinematics")
        #     print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        #     print("Caught expected error: {}".format(ex))
        #     return False

        # print("Joint ID : Joint Angle")
        # joint_identifier = 0
        # for joint_angle in computed_joint_angles.joint_angles :
        #     print(joint_identifier, " : ", joint_angle.value)
        #     joint_identifier += 1

        # return True
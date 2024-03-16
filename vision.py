import sys
# For image processing
import numpy as np
import cv2
from connect import RobotAPI
from scipy.spatial.transform import Rotation
class BallDetector:
    """
    This class uses the arm's camera to find a green ball and then calculate its center coordinates in the global frame
    """
    def __init__(self, robot_api: RobotAPI):
        """
        :param robot_api: object that has established a connection to the arm
        """
        # Save the current attached api
        self.robot_api = robot_api

        # You don't need a connection to access the stream itself, just the ip
        self.camera_stream = f"rtsp://{self.robot_api.ip}/color"
        # Video capture with opencv so you can process the images
        self.video_capture = cv2.VideoCapture(self.camera_stream)

        # You need the matrix to calculate point distance
        intrinsics = self.robot_api.get_intrinsic_vision_parameters()
        self.intrinsic_matrix = np.array([
            [intrinsics.focal_length_x, 0, intrinsics.principal_point_x],
            [0, intrinsics.focal_length_y, intrinsics.principal_point_y],
            [0, 0, 1]
        ])

        # Transform from the tool frame to the camera frame (hardcoded. Don't change)
        self.tool_to_camera_translation = np.array([0, 0.056, -0.123])
        self.tool_to_camera_rotation = np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]
        ])

        # Image processing parameters
        # Blur
        self.kernel_size = (11, 11)
        self.sigma = 0
        # Green color threshold
        # Very ad hoc values... might need to be changed depending on lighting
        self.greenLower = (36, 25, 25)
        self.greenUpper = (70, 255, 255)
        # Erosion/dilation param
        self.erosion_iterations = 20
        self.dilation_iterations = 20

        # Ball geometry parameters
        self.ball_radius = 0.03
        self.ball_height = 0.05
        self.kinova_support = 0.02  # height of black base under the arm



    def find_object(self, display_view: bool = True):
        """
        This function will look for a green ball in the camera video stream
        :param display_view: Whether opencv should show a screen with the tracked ball
        :return: Position of the ball center w.r.t global frame
        """
        try:
            # Get the current frame from the video (there might be a tiny delay)
            _, image = self.video_capture.read()

            # Blur and convert to HSV
            blurred = cv2.GaussianBlur(image, self.kernel_size, self.sigma)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # Create a mask of color green
            mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
            mask = cv2.erode(mask, None, self.erosion_iterations)
            mask = cv2.dilate(mask, None, self.dilation_iterations)

            # Find the right contours
            cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
                    # If the user wants the view displayed
                    if display_view:
                        cv2.imshow("Tracked ball", image)
                        cv2.waitKey(1)

                    # Calculate the direction of the ray in the camera frame
                    px = x
                    py = y
                    cx = self.intrinsic_matrix[0, 2]
                    cy = self.intrinsic_matrix[1, 2]
                    cz = self.intrinsic_matrix[2, 2]
                    fx = self.intrinsic_matrix[0, 0]
                    fy = self.intrinsic_matrix[1, 1]
                    pc0 = np.array([
                        (px - (cx / cz)) * (cz / fx),
                        (py - (cy / cz)) * (cz / fy),
                        1
                    ])

                    # Get the transformation from the base to the tool and from that compute base to camera
                    # I don't know how to get base to camera directly with the API...

                    base_to_tool = self.robot_api.base.GetMeasuredCartesianPose()
                    base_to_tool_translation = np.array([base_to_tool.x, base_to_tool.y, base_to_tool.z])
                    base_to_tool_rotation = Rotation.from_euler('ZYX',
                                                                np.array([base_to_tool.theta_z, base_to_tool.theta_y, base_to_tool.theta_x])*np.pi/180)
                    base_to_camera_rotation = base_to_tool_rotation.as_matrix()@self.tool_to_camera_rotation
                    base_to_camera_translation = base_to_tool_rotation.as_matrix()@self.tool_to_camera_translation + base_to_tool_translation


                    # Solve the camera ambiguity
                    # Monocular cameras can't pinpoint depth. This parameter will be estimated by knowing the target height in global coordinates
                    center_height = self.ball_height - self.ball_radius
                    cam_z = ((center_height - self.kinova_support - base_to_camera_translation[2]) /
                             (base_to_camera_rotation[2, :] @ pc0))
                    # Get the point in the camera frame
                    cam_point = cam_z * pc0
                    # Convert to global
                    global_point = base_to_camera_rotation @ cam_point + base_to_camera_translation
                    return global_point

        except:
            print('An error has occurred')

    def end_vision(self):
        """
        Call this to end vision activities
        :return:
        """
        self.video_capture.release()
        cv2.destroyAllWindows()


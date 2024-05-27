#!/usr/bin/env python3
import rospy
import rospkg
import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Aruco:
    def __init__(self):
        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        # Side length of the ArUco marker in meters 
        self.aruco_marker_side_length = 0.0365

        # Calibration parameters yaml file
        self.camera_calibration_parameters_filename = rospack.get_path('youbot_realsense') + "/calibration/calibration.yaml"
        
        # Load the camera parameters from the saved file
        self.cv_file = cv.FileStorage(self.camera_calibration_parameters_filename, cv.FILE_STORAGE_READ) 
        self.mtx = self.cv_file.getNode('K').mat()
        self.dst = self.cv_file.getNode('D').mat()
        self.cv_file.release()

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv.aruco.detectMarkers(cv_image, self.arucoDict, parameters=self.arucoParams)
        
        # Check that at least one ArUco marker was detected
        if ids is not None:
            # Draw a square around detected markers in the video frame
            cv.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv.aruco.estimatePoseSingleMarkers(
            corners,
            self.aruco_marker_side_length,
            self.mtx,
            self.dst)
                
            # Print the pose for the ArUco marker
            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, 
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera
            for i, id in enumerate(ids):
                # Store the translation (i.e. position) information
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]

                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                    
                # Quaternion format     
                transform_rotation_x = quat[0] 
                transform_rotation_y = quat[1] 
                transform_rotation_z = quat[2] 
                transform_rotation_w = quat[3] 
                    
                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, 
                                                                transform_rotation_y, 
                                                                transform_rotation_z, 
                                                                transform_rotation_w)
                    
                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                    
                # Draw the axes on the marker
                cv.drawFrameAxes(cv_image, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)
            
        # Display the resulting frame
        cv.imshow("Image window", cv_image)

        if cv.waitKey(3) == ord('q'):
            rospy.signal_shutdown("Closing windows")
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians


def main():
    Aruco()

    rospy.init_node('aruco', anonymous=True)

    rospy.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
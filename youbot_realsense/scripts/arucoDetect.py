#!/usr/bin/env python3
import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Aruco:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
        self.arucoParams = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv.aruco.detectMarkers(cv_image, self.arucoDict, parameters=self.arucoParams)
        
        if ids is not None:
            cv.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv.imshow("Image window", cv_image)

        if cv.waitKey(3) == ord('q'):
            rospy.signal_shutdown("Closing windows")

def main():
    Aruco()

    rospy.init_node('aruco', anonymous=True)

    rospy.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
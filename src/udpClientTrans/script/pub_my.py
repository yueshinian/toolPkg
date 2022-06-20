#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


def main():
    cvbridge = CvBridge()
    rospy.init_node("image_pub", log_level=rospy.DEBUG)
    pub_depth = rospy.Publisher("/depth/image", Image, queue_size=10)
    depth = np.ones((320, 180, 1), dtype=np.float32)
    depth_pub = cvbridge.cv2_to_imgmsg(depth, "32FC1")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_depth.publish(depth_pub)
        rate.sleep()


if __name__ == '__main__':
    main()

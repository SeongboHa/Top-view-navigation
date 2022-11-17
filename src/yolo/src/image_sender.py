#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class grab():

    def __init__(self):
        bridge = CvBridge()
        img_pub = rospy.Publisher('/captured_img', Image, queue_size=10)

        while not rospy.is_shutdown():
            img = cv2.imread('/home/riboha/catkin_ws/src/yolo/src/drone_view3.png')
            y = 90
            x = 2500
            img = img[y:y+928, x:x+928]
            img = bridge.cv2_to_imgmsg(img)
            img_pub.publish(img)



if __name__ == "__main__":
    rospy.init_node("img_sender")
    grab()

    

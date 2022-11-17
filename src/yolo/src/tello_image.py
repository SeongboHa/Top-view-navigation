#!/usr/bin/env python

from djitellopy import tello
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mapper.msg import detections

class manager():
    def __init__(self):
        self.Tello = tello.Tello()
        self.Tello.connect()
        print("start battery", self.Tello.get_battery())
        self.Tello.streamon()
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("/captured_img", Image, queue_size=None)
  
    def image_sender(self):
        img_ = self.Tello.get_frame_read().frame
        img_ = img_[0:720,120:840]          # 720 960 -> 720 720
        img_ = cv2.resize(img_, (640,640))  # 720 720 -> 640 640
        self.img = img_.copy()
        imgg = self.bridge.cv2_to_imgmsg(img_)
        self.image_pub.publish(imgg)
        rospy.Subscriber("detections", detections, self.detection_input)

    def detection_input(self, data):
        detections = data.detections
        w = 0
        for number in range(len(detections)):
            if detections[number].name == 'person':
                if w == 0:
                    w = detections[number].xmax - detections[number].xmin
                    h = detections[number].ymax - detections[number].ymin
                    cx = (detections[number].xmax + detections[number].xmin) / 2
                    cy = (detections[number].ymax + detections[number].ymin) / 2
                elif w*h < (detections[number].xmax - detections[number].xmin) * (detections[number].ymax - detections[number].ymin):
                    w = detections[number].xmax - detections[number].xmin
                    h = detections[number].ymax - detections[number].ymin
                    cx = (detections[number].xmax + detections[number].xmin) / 2
                    cy = (detections[number].ymax + detections[number].ymin) / 2
        if w != 0:
            self.tracking(cx, cy, w, h)

        cv2.imshow('tracking', self.img)
        cv2.waitKey(1)

    def tracking(self, cx, cy, w, h):
        cv2.rectangle(self.img, (int(cx - w/2),int(cy - h/2)),(int(cx + w/2),int(cy + h/2)),(255,0,0),2)
        if cx > 350:
            #self.Tello.send_rc_control(0,0,0,20) # 우회전
            print("우회전")
        if cx < 290:
            #self.Tello.send_rc_control(0,0,0,-20) # 좌회전
            print("좌회전")
        
if __name__ =='__main__':
    rospy.init_node("tello")
    a = manager()
    while not rospy.is_shutdown():
        a.image_sender()
#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

class cleaner():
    def __init__(self):

        map_topic = "/map"

        self.iter = 0
        self.server = rospy.service(map_topic, GetMap, self.clear)
        self.detections_pub = rospy.Publisher(map_topic, OccupancyGrid, queue_size=10)
        rospy.spin()

    def clear(self, req):
        self.iter += 1
        print(self.iter)
        if self.iter == 5:
            cleaned_map_topic = OccupancyGrid()
            count = len(data.data)
            cleaned_map_topic.data = np.full(count,-1)
            self.detections_pub.publish(cleaned_map_topic)
            self.iter = 0
        

if __name__ =='__main__':
    rospy.init_node("map_cleaner")
    cleaner()
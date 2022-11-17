#!/usr/bin/env python

import rospy
from mapper.msg import detections
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.srv import GetMap, GetMapResponse
from sensor_msgs.msg import Image
import cv2
import ros_numpy
import numpy as np

class mapper():
    def __init__(self):

        self.image_size = 928
        self.detections = detections()

        rospy.Subscriber("detections", detections, self.detection_input)
        rospy.Subscriber('/top_img', Image, self.image_input)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=2)
        self.staticmap_srv = rospy.Service("/static_map", GetMap, self.mapping2)

        rospy.spin()

    def detection_input(self, data):
        self.detections = data
        self.mapping_base()

    def image_input(self, data):
        # input image
        image = ros_numpy.numpify(data)
        self.image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def mapping_base(self):
        detections = self.detections.detections
        input_image = self.image
        # 출력될 map
        created_map = np.full((self.image_size,self.image_size), -1)
        # mask 선언
        mask = np.zeros((self.image_size,self.image_size), dtype = np.uint8)
        # mask 만들기
        for number in range(len(detections)):
            for i in range(detections[number].xmin, detections[number].xmax):
                for j in range(detections[number].ymin, detections[number].ymax):
                    mask[j, i] = 255

        #마스크 결과 이미지로 쓸 것
        result_map = np.zeros((self.image_size,self.image_size), dtype = np.uint8)
        # 지역 임계 이진화 적용
        # middle = cv2.adaptiveThreshold(input_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        #                         cv2.THRESH_BINARY_INV, 5, 5)
        _, middle = cv2.threshold(input_image, 130, 255, cv2.THRESH_BINARY_INV)

        # kernel = np.ones((3, 3), np.uint8)
        # middle = cv2.dilate(middle, kernel, iterations=1)

        # 마스크 씌워서 객체 영역만 남기기
        # 결과 : result_map
        cv2.copyTo(middle, mask, result_map)

        # map으로 만들기
        for i in range(self.image_size):
            for j in range(self.image_size):
                if result_map[i, j] == 255:
                    created_map[i, j] = 100

        created_map = cv2.rotate(created_map, cv2.ROTATE_90_COUNTERCLOCKWISE)
        created_map = cv2.flip(created_map, 1)
        # created_map = cv2.flip(created_map, 0)

        grid = OccupancyGrid()

        # /map, /map_metadata 공통 값들 입력
        resolution = 0.0045
        width = self.image_size
        height = self.image_size
        position_x = 0        # 감소 - 아래
        position_y = 0      # 증가 - 좌
        orientation_w = 1.0

        # /map topic 값들 넣음
        grid.data = created_map.flatten()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = position_x
        grid.info.origin.position.y = position_y
        grid.info.origin.orientation.w = orientation_w

        self.map_pub.publish(grid)


    def mapping(self):
        detections = self.detections.detections
        created_map = np.full((self.image_size,self.image_size), -1)
        for number in range(len(detections)):
            for i in range(detections[number].xmin, detections[number].xmax):
                for j in range(detections[number].ymin, detections[number].ymax):
                    created_map[j,i] = 100

        created_map = cv2.rotate(created_map, cv2.ROTATE_90_COUNTERCLOCKWISE)
        created_map = cv2.flip(created_map, 1)

        grid = OccupancyGrid()
        

        # /map, /map_metadata 공통 값들 입력
        resolution = 0.02
        width = self.image_size
        height = self.image_size
        position_x = -1.0
        position_y = -1.0
        orientation_w = 1.0

 
        # /map topic 값들 넣음
        grid.data = created_map.flatten()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = position_x
        grid.info.origin.position.y = position_y
        grid.info.origin.orientation.w = orientation_w

        self.map_pub.publish(grid)


    def mapping2(self, request):
        detections = self.detections.detections
        created_map = np.full((self.image_size,self.image_size), -1)
        for number in range(len(detections)):
            for i in range(detections[number].xmin, detections[number].xmax):
                for j in range(detections[number].ymin, detections[number].ymax):
                    created_map[j,i] = 100

        grid = OccupancyGrid()
        metadata_topic = MapMetaData()
        map_srv = GetMapResponse()
        

        # /map, /map_metadata 공통 값들 입력
        resolution = 0.025
        width = self.image_size
        height = self.image_size
        position_x = 3.0
        position_y = -1.0
        orientation_w = 1.0

    
        # /map topic 값들 넣음
        grid.data = created_map.flatten()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = position_x
        grid.info.origin.position.y = position_y
        grid.info.origin.orientation.w = orientation_w

        # /map srv 값들 넣음

        map_srv.map = grid

        # /map_metadata 값들 넣음
        metadata_topic.resolution = resolution
        metadata_topic.width = width
        metadata_topic.height = height
        metadata_topic.origin.position.x = position_x
        metadata_topic.origin.position.y = position_y
        metadata_topic.origin.orientation.w = orientation_w

        # self.meta_pub.publish(metadata_topic)
        self.map_pub.publish(grid)

        # cv2.imshow("demo", created_map)
        # cv2.waitKey(1)

        return map_srv


if __name__ =='__main__':
    rospy.init_node("mapper")
    mapper()
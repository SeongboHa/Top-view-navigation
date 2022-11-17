import cv2
import numpy as np


def dotter(map, start_point):
    map[start_point] = 1
    map[start_point[0]+30, start_point[1]+40] = 16
    map[start_point[0]-10, start_point[1]+70] = 32

map1 = np.zeros((500,500))
map2 = np.zeros((500,500))

dotter(map1, (250,250))
dotter(map2, (100,110))


while True:
    output = cv2.hconcat((map1, map2))
    cv2.imshow("output", output)
    cv2.waitKey(1)

import cv2

while True:
    img = cv2.imread('/home/riboha/catkin_ws/src/yolo/src/drone_view3.png')
    y = 90
    x = 2500
    img = img[y:y+928, x:x+928]

    cv2.imshow("output", img)
    cv2.waitKey(1)
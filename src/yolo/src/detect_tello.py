#!/usr/bin/env python

import time

import rospy
from sensor_msgs.msg import Image
import ros_numpy

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from yolo.msg import detection, detections


from models.experimental import attempt_load
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

class detector():
    def __init__(self):

        weights = 'yolov7.pt'
        self.save_txt = 0
        self.imgsz = 928
        self.img_size = 928
        trace = True
        self.augment = 0

        self.save_img = 0  # save inference images

        # Initialize
        set_logging()
        self.device = select_device('')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check img_size
        
        if trace:
            self.model = TracedModel(self.model, self.device, self.imgsz)

        if self.half:
            self.model.half()  # to FP16``
        
        # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=self.device)['model']).to(self.device).eval()

        #self.view_img = check_imshow()
        self.view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference
        
        # Get names and colors
        
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        
        # image message input
        # self.image_topic = '/camera/rgb/image_raw'
        self.image_topic = '/top_img'
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.detect)

        self.detections_pub = rospy.Publisher("detections", detections, queue_size=None)
        
        rospy.spin()

    def detect(self, data):

        t0 = time.time()
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        # except CvBridgeError as e:
        #     print(e)

        old_img_w = old_img_h = self.imgsz
        old_img_b = 1

        #cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        cv_image = ros_numpy.numpify(data)

        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = self.letterbox(cv_image, self.img_size, stride=self.stride)[0]

        # 메모리 비우기
        # gc.collect()
        # torch.cuda.empty_cache()

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                self.model(img, augment=self.augment)[0]

        # Inference
        with torch.no_grad():
            t1 = time_synchronized()
            pred = self.model(img, augment=self.augment)[0]
            t2 = time_synchronized()

        # Apply NMS
        conf_thres = 0.20
        iou_thres = 0.40
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=None, agnostic=0)
        t3 = time_synchronized()

        # Apply Classifier
        # if self.classify:
        #     pred = apply_classifier(pred, self.modelc, img, im0s)

        # Process detections

        
        detections_result = detections()
        
        for i, det in enumerate(pred):  # detections per image
            im0 = cv_image.copy()
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # # Print results
                # for c in det[:, -1].unique():
                #     n = (det[:, -1] == c).sum()  # detections per class
                #     s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    # if self.save_txt:  # Write to file
                    #     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #     line = (cls, *xywh, conf)  # label format
                    #     with open(txt_path + '.txt', 'a') as f:
                    #         f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if self.save_img or self.view_img:  # Add bbox to image
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        #print( '(' , int(xyxy[0]), ', ', int(xyxy[1]), ') , (', int(xyxy[2]), ', ', int(xyxy[3]), ')')
                        plot_one_box(xyxy, im0, label=label, color=(100,100,255), line_thickness=1)
                        # plot_one_box(xyxy, im0, label='obstacle', color=(100,100,255), line_thickness=1)
                    detection_result = detection()
                    detection_result.xmin = int(xyxy[0])
                    detection_result.ymin = int(xyxy[1])
                    detection_result.xmax = int(xyxy[2])
                    detection_result.ymax = int(xyxy[3])
                    detection_result.name = self.names[int(cls)]
                    detection_result.probability = float(conf)
                    #rospy.loginfo(detection_result)
                    detections_result.detections.append(detection_result)

            # Print time (inference + NMS)
            print(f'Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Stream results
            if self.view_img:
                cv2.imshow('output', im0)
                cv2.waitKey(1)  # 1 millisecond
            #print(detections_result)
            self.detections_pub.publish(detections_result)
        
        print(f'Done. ({time.time() - t0:.3f}s)')

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)


if __name__ == '__main__':
    rospy.init_node("detector_ros")
    with torch.no_grad():
        detector()
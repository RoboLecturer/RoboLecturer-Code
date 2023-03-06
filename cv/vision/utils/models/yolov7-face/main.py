### python main.py --weights YOLO/weights/yolov7s-face.pt --save-txt --nosave

import argparse
from YOLO.detect import detect, config_net

import argparse
import time
from pathlib import Path

import os
import copy
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from YOLO.models.experimental import attempt_load
from YOLO.utils.datasets import LoadStreams, LoadImages
from YOLO.utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from YOLO.utils.plots import colors, plot_one_box
from YOLO.utils.torch_utils import select_device, load_classifier, time_synchronized
from YOLO.utils.general import check_requirements

parser = argparse.ArgumentParser()
parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
parser.add_argument('--source', type=str, default='YOLO/data/images', help='source')  # file/folder, 0 for webcam
parser.add_argument('--img-size', nargs= '+', type=int, default=640, help='inference size (pixels)')
parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
parser.add_argument('--view-img', action='store_true', help='display results')
parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
parser.add_argument('--save-txt-tidl', action='store_true', help='save results to *.txt in tidl format')
parser.add_argument('--save-bin', action='store_true', help='save base n/w outputs in raw bin format')
parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
parser.add_argument('--augment', action='store_true', help='augmented inference')
parser.add_argument('--update', action='store_true', help='update all models')
parser.add_argument('--project', default='runs/detect', help='save results to project/name')
parser.add_argument('--name', default='exp', help='save results to project/name')
parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
parser.add_argument('--line-thickness', default=1, type=int, help='bounding box thickness (pixels)')
parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
parser.add_argument('--kpt-label', type=int, default=5, help='number of keypoints')
opt = parser.parse_args()
print(opt)
check_requirements(exclude=('tensorboard', 'pycocotools', 'thop'))

model, imgsz, stride = config_net(opt=opt)

# Replace for the camera loop
for i in range(10):
    frame = cv2.imread('YOLO/data/images/selfie_01.jpg')

    # Returns the list of all faces coordinates
    faces_coords = detect(opt, frame, model, imgsz, stride)
    #print(faces_coords)
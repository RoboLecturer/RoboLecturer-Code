import argparse
import time
from pathlib import Path

import os
import copy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random

from utils.models.YOLOface.models.experimental import attempt_load
from utils.models.YOLOface.utils.datasets import LoadStreams, LoadImages, letterbox
from utils.models.YOLOface.utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.models.YOLOface.utils.plots import colors, plot_one_box
from utils.models.YOLOface.utils.torch_utils import select_device, load_classifier, time_synchronized

#cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
#cap.set(3, 1920)
#cap.set(4, 1080)

def config_net(opt):
    source, weights, view_img, save_txt, imgsz, save_txt_tidl, kpt_label = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, opt.save_txt_tidl, opt.kpt_label
    #save_img = not opt.nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    #save_dir = increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok)  # increment run
    #(save_dir / 'labels' if (save_txt or save_txt_tidl) else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu' and not save_txt_tidl  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    if isinstance(imgsz, (list,tuple)):
        assert len(imgsz) ==2; "height and width of image has to be specified"
        imgsz[0] = check_img_size(imgsz[0], s=stride)
        imgsz[1] = check_img_size(imgsz[1], s=stride)
    else:
        imgsz = check_img_size(imgsz, s=stride)  # check img_size
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    
    return model, imgsz, stride

def detect(opt, frame, model, imgsz, stride):
    source, weights, view_img, save_txt, imgsz, save_txt_tidl, kpt_label = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, opt.save_txt_tidl, opt.kpt_label
    device = select_device(opt.device)
    half = device.type != 'cpu' and not save_txt_tidl  # half precision only supported on CUDA
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    save_img = not opt.nosave and not source.endswith('.txt')  # save inference images

    t0 = time.time()
#for path, img, im0s, vid_cap in dataset:
    im0s = frame
    #im0s = cv2.imread('YOLO/data/images/classroom_01.jpg')
    #print(img)
    img = letterbox(im0s, imgsz, stride=stride, auto=False)[0]

    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)


    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = model(img, augment=opt.augment)[0]
    #print(pred[...,4].max())
    # Apply NMS
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms, kpt_label=kpt_label)
    t2 = time_synchronized()

    # Apply Classifier
    #if classify:
        #pred = apply_classifier(pred, modelc, img, im0s)

    # Process detections
    face = []
    faces_coords = np.array([])
    for i, det in enumerate(pred):  # detections per image
        #if webcam:  # batch_size >= 1
            #p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), dataset.count
        #else:
            #p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)

        s, im0 = '', im0s.copy()
        #p = Path(p)  # to Path
        #save_path = str(save_dir / p.name)  # img.jpg
        #txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
        #s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        if len(det):
            # Rescale boxes from img_size to im0 size
            scale_coords(img.shape[2:], det[:, :4], im0.shape, kpt_label=False)
            scale_coords(img.shape[2:], det[:, 6:], im0.shape, kpt_label=kpt_label, step=3)

            # Print results
            for c in det[:, 5].unique():
                n = (det[:, 5] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            xyxy = reversed(det[:, :4])
            # Write results
            #for det_index, (*xyxy, conf, cls) in enumerate(reversed(det[:,:6])):
            #    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).numpy()  # normalized xywh

            face.append(np.array(xyxy, dtype=np.float32))
            face = face[0]
            for det_index, face_detected in enumerate(face):
                kpts = det[det_index, 6:]
                step = 3
                num_kpts = len(kpts) // step
                coords = []
                for k_id in range(num_kpts):
                    #print(kpts)
                    x_coord, y_coord = kpts[step * k_id].numpy(), kpts[step * k_id + 1].numpy()
                    coords.append(np.array([x_coord, y_coord]))
                    #face = np.insert(face, det_index, np.array([x_coord, y_coord])).reshape(-1, 14)
                    #print(face)
                    #print(temp)
                face = np.append(face_detected, coords)
                faces_coords = np.append(faces_coords, face).reshape(-1, 14)
                #print(actual_faces)
                #face.insert(det_index, x_coord)
                #face.insert(det_index, y_coord)
                #print(xywh)
                #with open(txt_path + '.txt', 'a') as f:
                #f.write(('%g ' * len(line)).rstrip() % line + '\n')

            if save_img or opt.save_crop or view_img:  # Add bbox to image
                c = int(cls)  # integer class
                label = None if opt.hide_labels else (names[c] if opt.hide_conf else f'{names[c]} {conf:.2f}')
                kpts = det[det_index, 6:]
                plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=opt.line_thickness, kpt_label=kpt_label, kpts=kpts, steps=3, orig_shape=im0.shape[:2])
                    #cv2.imwrite('runs/detect/exp2/labels/test_01.jpg', im0)
                    #if opt.save_crop:
                        #save_one_box(xyxy, im0s, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

            #if save_txt_tidl:  # Write to file in tidl dump format
                #for *xyxy, conf, cls in det_tidl:
                    #xyxy = torch.tensor(xyxy).view(-1).tolist()
                    #line = (conf, cls,  *xyxy) if opt.save_conf else (cls, *xyxy)  # label format
                    #with open(txt_path + '.txt', 'a') as f:
                        #f.write(('%g ' * len(line)).rstrip() % line + '\n')

        # Print time (inference + NMS)
        #print(f'{s}Done. ({t2 - t1:.3f}s)')

        # Stream results
        #if view_img:
            #cv2.imshow(str(p), im0)
            #cv2.waitKey(1)  # 1 millisecond

        # Save results (image with detections)
        #if save_img:
            #print(save_path)
            #if dataset.mode == 'image':
                #cv2.imwrite(save_path, im0)
            #else:  # 'video' or 'stream'
                #if vid_path != save_path:  # new video
                    #vid_path = save_path
                    #if isinstance(vid_writer, cv2.VideoWriter):
                        #vid_writer.release()  # release previous video writer
                    #if vid_cap:  # video
                        #fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        #w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        #h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    #else:  # stream
                        #fps, w, h = 30, im0.shape[1], im0.shape[0]
                        #save_path += '.mp4'
                    #vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                #vid_writer.write(im0)

    #if save_txt or save_txt_tidl or save_img:
        #s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt or save_txt_tidl else ''
        #print(f"Results saved to {save_dir}{s}")

    #print(f'Done. ({time.time() - t0:.3f}s)')
    return faces_coords



# yolov7-face


#### Dataset

[WiderFace](http://shuoyang1213.me/WIDERFACE/)

[yolov7-face-label](https://drive.google.com/file/d/1FsZ0ACah386yUufi0E_PVsRW_0VtZ1bd/view?usp=sharing)

#### Demo

![](data/images_orig/result.jpg)

### Installation

Run
'''
pip install -r requirements.txt
'''
OBS: not all packages are included in the file above. Install manually if required.

### Predict

For Pepper, change the variable '''frame''' in '''main.py''' to camera frame and run
'''
python main.py --weights YOLO/weights/yolov7s-face.pt --save-txt --nosave
'''

For tests, uncomment line 149 from '''detect.py''' and run
'''
python main.py --weights YOLO/weights/yolov7s-face.pt --save-txt
'''
It will save the result image in the folder '''runs\detect\exp2'''

#### References

* [https://github.com/deepcam-cn/yolov5-face](https://github.com/deepcam-cn/yolov5-face)

* [https://github.com/derronqi/yolov7-face](https://github.com/derronqi/yolov7-face)

* [https://github.com/WongKinYiu/yolov7](https://github.com/WongKinYiu/yolov7)

* [https://github.com/TexasInstruments/edgeai-yolov5/tree/yolo-pose](https://github.com/TexasInstruments/edgeai-yolov5/tree/yolo-pose)

* [https://github.com/ppogg/YOLOv5-Lite](https://github.com/ppogg/YOLOv5-Lite)

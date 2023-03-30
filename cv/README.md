# CV Documentation

There are three main aspects in this module:
- Face and landmark detection - Done by YOLOv7 model.
- Hand Detection - Performed by the MediaPipe Hand model.
- Engagement - Compuuted by a deterministic function.

All of these components have been integrated into the ```main.py``` script that runs with the PepperAPI. To test the same code without the API, please run ```test_main.py```. For running any of these, please run the following command:

```python3 script.py --weights utils/models/YOLOface/weights/yolov7s-face.pt --save-txt --nosave```


## Contents

- [Camera Setup](#Camera-Setup)
- [Face Detection](#Face-Detection) 
- [Hand Raise Detection](#Hand-Raise-Detection)
- [Engagement](#Engagement)
- [Contributions](#Contributions)


## Camera-Setup

The camera setup is made with two functions defined in the main scripts:

```python

def set_up_camera():
    camera = cv2.VideoCapture("/dev/video6")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("H", "J", "P", "G"))
    camera.set(3, 1280)
    camera.set(4, 720)
    return camera


def get_camera_input(camera):
    ret, frame = camera.read()
    return frame

```

Firstly, the camera has to be set up with the first function. Make sure that the variable inside the ```cv2.VideoCapture()``` method is compatible with your device and computer due to that a different value might work for you. If you are using a Linux-based system, you can use the command ```v4l2-ctl --list-devices``` to see the list of devices detected by your PC. The line ```camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("H", "J", "P", "G"))``` was used due to the issues with UTM was making OpenCV to open frames flipped and in a grayscale. Then, the ```get_camera_input``` function is used inside the loops to access frames.


## Face-Detection

For the main face detection model function ```face_engagement_detection```:

Inputs:
- **face_model** (model) - YOLOv7 model for face detection.
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function.
- **opt** (parser.parse_args()) - arguments parsed when calling the script.
- **imgsz** - size of the frame.
- **stride** - stride.

Outputs:
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function.
- **engagement_record** (list) - an array of all computed engagement scores.


For the face and landmark detection function ```detect```:

Inputs:
- **opt** (parser.parse_args()) - arguments parsed when calling the script.
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function.
- **face_model** (model) - YOLOv7 model for face detection.
- **imgsz** - size of the frame.
- **stride** - stride.

Outputs:
- **faces** (numpy.array) - an array of bounding boxes of detected faces and corresponding face landmarks.


Additional Notes:

YOLOv7 is used for the face detection with landmarks. Whenever you run the command mentioned above, the weights from ```yolov7s-face.pt``` are loaded which is one of three possible options. The other ones are ```yolov7-lite-s.pt``` and ```yolov7-lite-t.pt``` which are faster but  less accurate versions of the architecture. The main scripts first check for the compatibility of the requirements with:

```python

parser = argparse.ArgumentParser()
parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="model.pt path(s)")
parser.add_argument("--source", type=str, default="utils/models/YOLOface/data/images", help="source")
parser.add_argument("--img-size", nargs="+", type=int, default=640, help="inference size (pixels)")


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
check_requirements(exclude=("tensorboard", "pycocotools", "thop"))

```

Nextly, the network is configured with ```config_net(opt=opt)``` line which outputs: the model, image size and a stride. After the hand detection finishes its execution, all of these variables are forwarded to the function below:


```python

def face_engagement_detection(face_model, frame, opt, imgsz, stride, engagement_record=[0]):
    # get faces
    faces = detect(opt, frame, face_model, imgsz, stride) # YoloV7

    # display boxes around faces
    if len(faces) != 0: # if a face was detected.
        for (x1, y1, x2, y2) in faces[:, 0:4]:
            w = (x2 - x1)
            h = (y2 - y1)
            # Computing landmarks of a detected face and computing an engagement score
            landmark_results_detected = faces[:, 4:]
            engagement = engagement_from_landmarks(landmark_results_detected, frame, w, h)
            engagement_record.append(engagement)
        
    return frame, engagement_record

```

Which detects faces that are then forwarded to the functions calculating the engagement per detected face patch. For more details about these functions, please navigate to the corresponding section. What's also important to mention is the dimensionality of the ```faces``` variable which in fact not only contains detected faces, but also the corresponding landmarks. For a single detected face, the variable is going to have ```(1, 14)``` dimensions. First two numbers stand for the coordinates of the top left corner of a bounding box whereas the next two describe the placement of a bottom right corner. Thus, the format of bounding boxes here is ```PASCAL_VOC```. The other numbers form a sequence of (x, y) coordinates of five detected landmarks. If one whishes to change the format of the ```face``` array, please visit ```detect.py``` script. When executing the ```main.py``` script, the ```face_engagement_detection``` function is run inside a loop until 10 seconds had elpased. It is important to keep it at least 8 seconds long as the methods need some time to appropriatelty detect faces and correctly calculate the engagement.


## Hand-Raise-Detection

For the main hand detection function ```hand_detector```:

Inputs:
- **model** (mp.solutions.hands.Hands) - hand detection model supplied by MediaPipe.
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function. 

Outputs:
- **hand_landmarks** (numpy.array) - an array of hands detected as the ones that were raised to ask a question. If no hands is detected, then an array of zeros is returned.


Additional notes:

As mentioned above, the hand detection goes before the face detection and engagement calculation. The ```hand_detector``` function detects hands and attempts to classify whether it is raised for a query or not based on a gesture modelling. The calculations are done for each detected hand and for each finger excluding the thumb (which has 1 index its knuckle and 4 for its fingertip). The thresholding happens on the ```if metric < 0.48``` line. This hyperparameter for this process is free to be changed. Similarly to the face detection, the ```hand_detector``` function runs inside a time loop in the ```main``` function so that the system has sufficient time to detect hands correctly. This time, it only runs for 6 seconds which is suffientely long. 


## Engagement

The engagement is computed from the detected 2D landmarks. It is done with the function ```engagement_from_landmarks```:

Inputs:
- **landmark_results_detected** (numpy.array) - the array of all detected landmarks.
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function. 
- **w** (numpy.float) - width of the detected bounding box.
- **h** (numpy.float) - height of the detected bounding box.

Outputs:
- **frame** (numpy.array) - frame obtained from the ```get_camera_input``` function. 
- **engagement_record** (list) - an array of all computed engagement scores.

Additional Notes:

The ```engagement_record``` list is used to calculate a mean engagement that denotes the engagement of the classroom. This values is then input to a threshold to detrmine whether the audiance is engaged. The value of the threshold is free to be changed depenedning on a desired strictness and a setup.


## Contributions

The contributions of this module was distributed the following way:

- lukemkU - Face and 2D landmark detection.
- man-like-vinny - Hand raise detection.
- shreya-51 - UTM setup and engagement mesurement.
- MattG-bci - Integration of the CV module with the rest of the system, engagement measurement and hand raise detection.





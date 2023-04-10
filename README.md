# RoboLecturer
## Overview
This project is part of the Human-Centred Robotics module at Imperial College London's Department of Electrical Engineering.

This project develops RoboLecturer, a robot designed to deliver lectures to a small class of students using slides uploaded by the lecturer. Using Softbank Robotics' Pepper robot, several sub-modules were developed and integrated to allow RoboLecturer to:
- Interact with the students
- Answer their questions
- Collect metrics for evaluation of class performance

## Functions
RoboLecturer's lectures are carried out in four main steps according to the flow diagram.
<p align="center">
  <img src="./images/flowchart.jpg" alt="Flowchart" width="600px">
</p>

**0. Pre-processing**: The lecturer uploads slides on the Web Server, which are parsed for text and sent to NLP. NLP then generates lecture scripts for each slides and sends them to Speech, which converts the text to audio files.
| ![Slides generation](./images/slides-generation.gif) | ![Lecture generation](./images/lecture-generation.gif) |
|:--:|:--:|
| *Parsing text from uploaded slides* | *Generation of lecture script* |

**1. Lecture deliverance**: RoboLecturer delivers the lecture script for the current slide, engaging arm and hand movements to appear more natural.
<p align="center">
  <a href="https://user-images.githubusercontent.com/76771375/230804854-d28d2e5d-a292-4f30-9d59-a1f7dd6a69f4.mp4"><img src="./images/lecture.gif" alt="Flowchart" width="500px"></a>
  <br>
  <sup>(click for sound)</sup>
</p>

**2. Question and Answer**: After delivering the lecture, RoboLecturer asks for questions, scans for raised hands, and answers the students' questions.
| ![Pointing](./images/pointing.gif) | [![Answer](./images/answer.gif)](https://user-images.githubusercontent.com/76771375/230805940-b0c7e866-4229-4c2e-a831-295949f49a35.mp4) |
|:--:|:--:|
| *Pointing at student with raised hand* | *Snippet of answer to the question "What is the shape of our galaxy?" (click for sound)* |

**3. Engagement Detection**: After QnA, RoboLecturer scans the room for inattentiveness by detecting face engagement and high noise. If attentiveness is detected, the robot makes a joke, tells the students to quiet down, or starts a quiz to revitalize the lecture.
<p align="center">
  <img src="./images/engagement.jpg" alt="Flowchart" width="500px">
  <br>
  <sup>Detection of face engagement using YOLOv7</sup>
</p>

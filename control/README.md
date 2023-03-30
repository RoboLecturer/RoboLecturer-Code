# Control module

This folder contains the main **control.py** script, as well as dummy scripts for the differet modules used to test the whole system flow without the Pepper robot and manual input for number of hands detected or questions.

Folder structure:
- **```control.py```**: Main control script
- **```cv.py```**: Dummy CV script with user input for number of hands
- **```nlp.py```**: Dummy NLP script with user input for whether a QnA iteration is finished
- **```send_slides.py```**: Script to send dummy hardcoded slides (to NLP)
- **```speech.py```**: Dummy Speech script with user input for student questions
- **```take_control.py```**: Script to send a dummy signal for "TakeControl" back to Control after a quiz has been finished
- **```data/```**: Folder containing camera parameters (camera matrix and distance coefficients) used for pose estimation
- **```documentation/```**: Other documentation and guides for setting up ROS and Linux

## Contents
- [Pre-requisites](#pre-requisites)
- [Usage](#usage)
- [Functions](#functions)

## Pre-requisites
- Ubuntu 16 (for ROS Kinetic)
- ROS Kinetic
- Python 2.7
- [NaoQi Python SDK 2.5.10 (an older version is used as Pepper's software is not updated)](https://www.aldebaran.com/en/support/pepper-naoqi-2-9/downloads-softwares)
- OpenCV (```pip install opencv-python```) for ArUco detection and pose estimation

## Usage
Run the roscore, then the main Control script.
```
roscore # in a terminal
python control.py # in a different terminal
```
To run all the dummy scripts, open different terminals and run them.
```
# each run in a different terminal
python cv.py
python speech.py
python nlp.py
python send_slides.py
```

## Functions
Besides the main loop that all scripts follow, the Control script runs the following threads:
- **```Action.Listen```**: Continuously listen to requests for Pepper actuation - playing audio, pointing, or changing volume. The actuations are wrapped in Subscriber callbacks which subscribe to topics which other modules publish to when they run an ```Action.Request```.
- **```Info.Listen```**: Continuously subscribe to state updates sent by the CV module (to update states AnyQuestions and Attentiveness), Speech module (to update state NoiseLevel) and Control module (to update states Start and NoQuestions) and update the global state variables, which are referenced by ```Info.Send(State)``` to publish the state values to all the modules for synchronization.
- **```Info.Broadcast```**: Continuously publish on topic *change_slide* and *trigger_quiz* for the Web module to receive, as connection issues with the webserver meant that the messages had to be constantly published instead of being published once as they are when sent to other modules.
- **```Action.Localize```**: Continuously detect ArUco markers with Pepper's 2D camera and perform pose estimation and localization. This updates the global variable D (the robot's offset w.r.t the CV camera in pixels), which is used in the pointing algorithm.

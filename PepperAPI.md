# PepperAPI Documentation
The three functions are:
- **```Action.Request(name, params)```**: Request for Pepper to perform an action
- **```Info.Request(name, params)```**: Request to receive data from other modules or from Pepper
- **```Info.Send(name, params)```**: Request to send data to other modules

For each script in which you need to call the API, import the **PepperAPI** package and initialise it with the name of your module (e.g. **cv_module**). This name should remain the same for all scripts, or it'll trigger a port error.
```
import PepperAPI
from PepperAPI import Info

if __name__ == "__main__":
    PepperAPI.init("cv_module")
    Info.Request("TriggerHandDetection")
```

## Contents
- [Web](#web)
- [CV](#cv)
- [NLP](#nlp)
- [Speech](#speech)
- [Kinematics](#kinematics)
- [Control](#control)

## Web
#### Send
- **```Info.Send("Slides", {"text": myText})```**: Send slides text to NLP module  
  - **params** (*Dict*) : Slides text to be provided as String to key ```text```
- **```Info.Send("TakeControl")```**: Send signal ("1") to Control module to start next loop
  
#### Receive
- **```Info.Request("TriggerQuiz")```**: Receive signal ("1") to trigger quiz
- **```Info.Request("ChangeSlide")```**: Receive signal ("1") to change slide


## CV
#### Send
- **```Info.Send("RaisedHandInfo", params)```**: Send information about raised hand to Kinematics module 
  - **params** (*Dict*) : 
    - key ```bounding_box```: *Float* (x,y,w,h)
    - key ```frame_res```: *Int* (width, height) of frame
    - key ```confidence_score```: *Float* confidence score

- **```Info.Send("FaceInfo", params)```**: Send information about detected face to Kinematics module 
  - **params** (*Dict*) : 
    - key ```bounding_box```: *Float* (x,y,w,h)
    - key ```frame_res```: *Int* (width, height) of frame
    - key ```engagement_score```: *Float* engagement score

- **```Info.Send("TriggerNoiseDetection")```**: Send signal ("1") to Speech module to trigger noise detection when there are no raised hands detected
- **```Info.Send("TriggerQuiz")```**: Send signal ("1") to Web module to trigger quiz when inattentiveness is detected
- **```Info.Send("TriggerJoke")```**: Send signal ("1") to NLP module to trigger joke when inattentiveness is detected
- **```Info.Send("IncrementLoopCounter")```**: Send signal ("1") to Control module to increment loop counter when no inattentiveness is detected
  
#### Receive
- **```Info.Request("TriggerHandDetection")```**: Receive signal ("1") to start detecting raised hands
- **```Info.Request("TriggerAttentivenessDetection")```**: Receive signal ("1") to detecting for inattentiveness


## NLP
#### Send
- **```Info.Send("LectureScript", {"text": myText})```**: Send lecture script text to Speech module
  - **params** (*Dict*) : Script text to be provided as String to key ```text```

- **```Info.Send("Answer", {"text": myText})```**: Send QnA answer text to Speech module
  - **params** (*Dict*) : Answer text to be provided as String to key ```text```

- **```Info.Send("Joke", {"text": myText})```**: Send joke text to Speech module
  - **params** (*Dict*) : Joke text to be provided as String to key ```text```
  
#### Receive
- **```Info.Request("Question")```**: Receive speech-to-text of detected question from Speech module
  - **return** (*String*) : Question text

- **```Info.Request("TriggerJoke")```**: Receive signal ("1") to trigger joke to be sent to Speech module

## Speech
#### Send
- **```Info.Send("Question", {"text": myText})```**: Send QnA question text to NLP module
  - **params** (*Dict*) : Question text to be provided as String to key ```text```

- **```Action.Request("ALAudioPlayer", {"path": filepath})```**: Request for generated audio to be played by Pepper's speakers
  - **params** (*Dict*) : Filepath of audio file (e.g. "C:/Users/user/sample.mp3" to be provided as String to key ```path```

- **```Info.Send("TriggerJoke")```**: Send signal ("1") to NLP module to trigger joke when high noise levels are detected
- **```Info.Send("TriggerAttentivenessDetection")```**: Send signal ("1") to CV to start detecting for inattentiveness when low noise levels are detected

#### Receive
- **```Info.Request("LectureScript")```**: Receive script for current slide from NLP module
  - **return** (*String*) : Script text

- **```Info.Request("Joke")```**: Receive joke text from NLP module
  - **return** (*String*) : Joke text

- **```Info.Request("Answer")```**: Receive QnA answer text from NLP module
  - **return** (*String*) : Answer text

- **```Info.Request("TriggerListen")```**: Receive signal ("1") to start listening to mic input
- **```Info.Request("TriggerNoiseDetection")```**: Receive signal ("1") to start detecting for noise


## Kinematics
#### Send
- **```Info.Send("TriggerHandDetection")```**: Send signal ("1") to CV module to start detecting for raised hands after Pepper has finished the script for the slide
- **```Info.Send("TriggerListen")```**: Send signal ("1") to Speech module to start listening to mic input after Pepper has pointed at the student
- **```Info.Send("ChangeSlide")```**: Send signal ("1") to Web module to change slide after Pepper has finished telling the joke

#### Receive
- **```Info.Request("RaisedHandInfo")```**
  - **return** (*Dict*) : 
    - key ```bounding_box```: *Float* (x,y,w,h)
    - key ```frame_res```: *Int* (width, height) of frame
    - key ```confidence_score```: *Float* confidence score


## Control
#### Send
- **```Info.Send("TriggerQuiz")```**: Send signal ("1") to Web module to trigger quiz when loop counter reaches threshold
- **```Info.Send("TriggerJoke")```**: Send signal ("1") to NLP module to trigger joke when loop counter reaches threshold
- **```Info.Send("ChangeSlide")```**: Send signal ("1") to Web module to change slide

#### Receive
- **```Info.Request("IncrementLoopCounter")```**: Receive signal ("1") to increment "no questions" loop counter

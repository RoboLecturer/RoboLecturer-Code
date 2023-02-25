# PepperAPI Documentation
The three functions are:
- **```Action.Request(name, params)```**: Request for Pepper to perform an action
- **```Info.Request(name, params)```**: Request to receive data from other modules or from Pepper
- **```Info.Send(name, params)```**: Request to send data to other modules

## Contents
- [Importing & Initialising](#importing--initialising)
- [Blocking transmission](#blocking-transmission)
- [APIs](#apis)
  - [Web](#web)
  - [CV](#cv)
  - [NLP](#nlp)
  - [Speech](#speech)
  - [Kinematics](#kinematics)
  - [Control](#control)

## Importing & Initialising
For each script in which you need to call the API, import the **PepperAPI** package and initialise it with the name of your module (e.g. **cv_module**). This name should remain the same for all scripts, or it'll trigger a port error.
```
import PepperAPI
from PepperAPI import Info

if __name__ == "__main__":
    PepperAPI.init("cv_module")
    Info.Request("TriggerHandDetection")
```

## Blocking transmission
- In general, the Publishers and Subscribers implemented by the API are blocking
- Publishers will wait for a subscriber to connect before they publish the msg
- Subscribers will also wait for data to be received before allowing subsequent code to run
- An exception is ```Info.Request("State",{"name":XXX})```. Because the states will be continually sent by the Control module, you will see that ```Info.Request("State")``` is often wrapped in a ```while``` loop to continually the state until it is updated

## APIs
### Web
#### Send
- **```Info.Send("Slides", {"text": myText})```**: Send slides text to NLP module  
  - **params** (*Dict*) : Slides text to be provided as *String* to key ```text```

- **```Info.Send("TakeControl")```**: Send signal ("1") to Control module to take back control after quiz has finished
  
#### Receive
- **```Info.Request("TriggerJokeOrQuiz")```**: Receive signal (*String*) "joke" or "quiz". If signal received is "quiz", trigger quiz

___
### CV
#### Send
- **```Info.Send("NumHands", {"value": numberOfHands})```**: Send number of hands to Kinematics before publishing hand info
  - **params** (*Dict*) : Number of hands to be provided as *Int* to key ```value```
  - 
- **```Info.Send("RaisedHandInfo", params)```**: Send information about raised hand to Kinematics module 
  - **params** (*Dict*) : 
    - key ```bounding_box```: *Float* (x,y,w,h)
    - key ```frame_res```: *Int* (width, height) of frame
    - key ```confidence_score```: *Float* confidence score

- **```Info.Send("State", {"AnyQuestion":"HandsRaised"/"NoHandsRaised"})```**: Update state ```AnyQuestions```
  - **params** (*Dict*) : New state "HandsRaised" or "NoHandsRaised" to be provided as *String* to key ```AnyQuestions```

- **```Info.Send("State", {"Attentiveness":"NotAttentive"/"Attentive"})```**: Update state ```Attentiveness```
  - **params** (*Dict*) : New state "NotAttentive" or "Attentive" to be provided as *String* to key ```Attentiveness```
  
#### Receive
- **```Info.Request("TriggerHandDetection")```**: Receive signal ("1") to start detecting raised hands

___
### NLP
#### Send
- **```Info.Send("LectureScript", {"text": myText})```**: Send lecture script text to Speech module
  - **params** (*Dict*) : Script text to be provided as *String* to key ```text```

- **```Info.Send("Answer", {"text": myText})```**: Send QnA answer text to Speech module
  - **params** (*Dict*) : Answer text to be provided as *String* to key ```text```

- **```Info.Send("Joke", {"text": myText})```**: Send joke text to Speech module
  - **params** (*Dict*) : Joke text to be provided as *String* to key ```text```

- **```Action.Request("ChangeVolume", {"cmd": "up"/"down"})```**: Request for volume to be increased/decreased
  - **params** (*Dict*) : Desired action "up"/"down" to be provided as *String* to key ```cmd```
  
#### Receive
- **```Info.Request("Question")```**: Receive speech-to-text of detected question from Speech module
  - **return** (*String*) : Question text

- **```Info.Request("TriggerJokeOrQuiz")```**: Receive signal (*String*) "joke" or "quiz". If signal received is "joke", send joke to Speech module

- **```Info.Request("TriggerJokeOrShutup")```**: Receive signal (*String*) "joke" or "shutup" to send joke or shutup to Speech module

___
### Speech
#### Send
- **```Info.Send("Question", {"text": myText})```**: Send QnA question text to NLP module
  - **params** (*Dict*) : Question text to be provided as *String* to key ```text```

- **```Action.Request("ALAudioPlayer", {"path": filepath})```**: Request for generated audio to be played by Pepper's speakers
  - **params** (*Dict*) : Filepath of audio file (e.g. "C:/Users/user/sample.mp3" to be provided as *String* to key ```path```

- **```Info.Send("State", {"NoiseLevel":"High"/"Low"})```**: Update state ```NoiseLevel```
  - **params** (*Dict*) : New state "High" or "Low" to be provided as *String to key ```NoiseLevel```

#### Receive
- **```Info.Request("LectureScript")```**: Receive script for current slide from NLP module
  - **return** (*String*) : Script text

- **```Info.Request("Joke")```**: Receive joke text from NLP module
  - **return** (*String*) : Joke text

- **```Info.Request("Joke")```**: Receive joke text from NLP module
  - **return** (*String*) : Joke text

- **```Info.Request("Answer")```**: Receive QnA answer text from NLP module
  - **return** (*String*) : Answer text

- **```Info.Request("TriggerListen")```**: Receive signal ("1") to start listening to mic input

___
### Kinematics
#### Send
- **```Info.Send("TriggerHandDetection")```**: Send signal ("1") to CV module to start detecting for raised hands after Pepper has finished the script for the slide
- **```Info.Send("TriggerListen")```**: Send signal ("1") to Speech module to start listening to mic input after Pepper has pointed at the student

#### Receive
- **```Info.Request("RaisedHandInfo")```**
  - **return** (*CVInfo msg*) Raised hand info wrapped in ROS msg

- **```Info.Request("TakeControl")```**: Receive signal ("1") to indicate that quiz has finished

___
### Control
#### Send
- **```Info.Send("TriggerJokeOrQuiz")```**: Send signal ("joke"/"quiz") to NLP & Web module to trigger joke or quiz when loop counter reaches threshold
- **```Info.Send("TriggerJokeOrShutup")```**: Send signal ("joke"/"quiz") to NLP module to trigger joke/shutup when loop counter reaches threshold

#### Receive
- **```Info.Request("State", params)```**: Receive state update
  - **params** (*Dict*) : 
    - keys ```<state_name>```: *String* <new_state>
    - keys ```<print>```: *boolean* False if no headers should be printed
  - **return** (*String*) : Value of queried state

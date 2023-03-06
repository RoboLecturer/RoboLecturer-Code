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
- [Developer docs](#developer-docs)
  - [Info.Send()](#infosend)
  - [Info.Request()](#inforequest)

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

## APIs
### Web
#### Send
- **```Info.Send("NumSlides", {"value": numberOfSlides})```**: Send number of slides to NLP before publishing slides text
  - **params** (*Dict*) : Number of slides to be provided as *Int* to key ```value```
 
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


- **```Info.Send("ChangeSlide", {"cmd": changeSlideCommand})```**: Send command to change slide - ```"increment|0"``` to increment the slide, ```"decrement|0"``` to decrement the slide and ```"goto|<slide_num>"``` to go to a slide num
  - **params** (*Dict*) : Command be provided as *String* to key ```cmd```
  
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
- **```Info.Send("ChangeSlide", {"cmd": changeSlideCommand})```**: Send command to change slide - ```"increment|0"``` to increment the slide, ```"decrement|0"``` to decrement the slide and ```"goto|<slide_num>"``` to go to a slide num
  - **params** (*Dict*) : Command be provided as *String* to key ```cmd```
- **```Info.Send("State", params)```**: Update state
  - **params** (*Dict*) : 
    - key ```<state_name>```: *String* New state value
    - (Optional) key ```print```: *boolean* False if no headers should be printed

#### Receive
- **```Info.Request("State", params)```**: Receive state update
  - **params** (*Dict*) : 
    - key ```name```: *String* Name of state to be queried
    - (Optional) key ```print```: *boolean* False if no headers should be printed
  - **return** (*String*) : Value of queried state
  
  
## Developer docs
### Info.Send()
1. Add the topic you want to publish to to **\_\_init.py\_\_** as constant.
2. Create publisher in **Publisher.py**
```
my_publisher = StringPublisher(MY_TOPIC, num_subscribers=2)
```
The ```num_subscribers``` argument defines how many modules should be listening before the message is published. Default is 1.

3. Add your api callback to **Info.py** under ```Send()```. 
```
if api_name == "MyApi":
    msg = api_params["msg"]
    my_publisher.publish(msg)
```
will result in the call ```Info.Send("MyApi", {"msg": "HelloWorld"}```.

### Info.Request()
1. Ensure your topic has been added to **\_\_init.py\_\_**.
2. Add your api callback to **Info.py** under ```Request()```.
```
if api_name == "MyApi":
    # Define what happens when msg is received
    def callback(msg):
        received_string = msg.data # actual string is in attr data
        Data.MyApi = received_string # saves the data so it can be returned
    
    # Create your subscriber
    StringSubscriber(MY_TOPIC, callback, listen=1)
    
    return Data.MyApi
```
- The ```listen``` argument takes in how many messages the subscriber should receive before unregistering. 
- If you need ```Info.Request()``` to return a value, add an attribute to the ```Data``` class in ```Request()``` and store your data there in your subscriber callback.
- The list of available subscriber (payload) types are in **Subscriber.py**.

## Action.Request()
The module will call ```Action.Request()```, which will publish a message to the Kinematics module which is calling ```Action.Listen()``` to perform a callback when the message is received.

1. Determine the NAOqi API needed to perform the desired callback. A list is [here](http://doc.aldebaran.com/2-1/naoqi/index.html). For example, Pepper's TTS is ALTextToSpeech. The desired method is probably ```say()```, which means that the module calling ```Action.Request()``` would need to supplement the message they want to be said as an argument.
```
Action.Request("ALTextToSpeech", {"value": "Hello World"})
```

2. Follow steps 1-2 of [Info.Send()](#infosend) to add your topic and create a publisher for it.

3. Add your block to **Action.py** under ```Request()```.
```
if api_name == "MyApi":
    string_to_say = api_params["value"]
    my_publisher.publish(string_to_say)
```

4. Now, define what happens when Kinematics receives the message (i.e. what Pepper should do). First, call the NAOqi module needed through a proxy. This is called under ```
Listen()``` of **Action.py**. 
```
try:
    broker = ALBroker(...)
    ...
    my_proxy = ALProxy("ALTextToSpeech")
```

5. Define the desired callback once the ```Action.Request()``` is received. ALTextToSpeech needs a message for its ```say()``` method.
```
def my_callback(msg):
    string_to_say = msg.data
    my_proxy.say(string_to_say)
```

6. Create a thread for your callback so it can run continuously.
```
thread_my_callback = threading.Thread(target=subscribe_listen, args=[lambda: StringSubscriber(MY_TOPIC, my_callback, listen=0)])
```

The ```subscribe_listen()``` function is predefined to allow your Subscriber to wait for the msg to be received (i.e. ```rospy.spin()```). The ```listen=0``` means your subscriber never unregisters.

7. Allow your thread to start and end by adding ```t.start()``` and ```t.join()``` where you see similar lines.

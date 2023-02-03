# ROS - Python Documentation
## Contents
1. [Overview](#1-overview)
2. [Setup](#2-setup)
- [Linux and ROS installation](#21-linux-and-ros-installation)
- [Setting up your workspace](#22-setting-up-your-workspace)
- [Setting ROS variables](#23-setting-ROS-variables)
3. [Running your scripts](#3-running-your-scripts)
- [Basic steps](#31-basic-steps)
- [Using the API](#32-using-the-api)

## 1. Overview
Install your version of ROS. ROS Noetic (Ubuntu 20.04) is recommended since it supports Python 3 officially. The default Python version for Ubuntu 20.04 is Python 3.8, but it should support Python 3.10 (see [here](https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/) to install). 

After installing ROS, [set up a workspace](#22-setting-up-your-workspace) and place your scripts inside. To ask Pepper to do something, or to receive some information from Pepper, use the API in your own scripts: call ```Action.Request()```,```Info.Request()``` or ```Info.Send()```. For example, this script requests for Pepper to say "Hello World".
```
#!/usr/bin/env python3
import PepperAPI
from PepperAPI import Action

if __name__ == "__main__":	
	PepperAPI.init()                                           # initialize the API	
	Action.Request("ALTextToSpeech", {"value":"Hello World"})  # request action
```

The list of Pepper's APIs can be found [here](http://doc.aldebaran.com/2-5/naoqi/index.html). API names to send/receive data between the subteams can be defined by you (e.g. RLContent).

## 2. Setup
### 2.1. Linux and ROS installation
See **[LinuxSetup.md](https://github.com/RoboLecturer/RoboLecturer-Code/blob/api/LinuxSetup.md)** for the full details.

### 2.2. Setting up your workspace
**Note:** if you load and run the image in the repo, a sample workspace and package has already been created.

The basic steps are:
```
mkdir -p ~/<workspace_name>/src         # create workspace folder with "src" folder inside
cd <workspace_name>/src
catkin_create_pkg <package_name> rospy  # create your package in the "src" folder
cd ..                                   # return to workspace folder
catkin_make                             # run catkin_make
source devel/setup.bash
```

Your Python scripts will be placed in your package folder. The **PepperAPI** package folder should be in the same directory as your Python scripts.
The final directory listing should be:
```
<workspace_name>
|    +----build
|    +----devel
|    |         setup.bash
|    \----src
|    |    \----<package name>
|    |    |    +----src
|    |    |         CMakeLists.txt
|    |    |         packages.xml
|    |    |    +----PepperAPI
|    |    |         myScript.py
```

Whenever scripts are updated, you should run ```catkin_make``` to rebuild your package in case your package dependencies have changed.
```
cd ~/<workspace_name>
catkin_make
source devel/setup.bash
```

### 2.3. Setting ROS variables
To connect other machines, your ROS variables need to be set accordingly:
- ```ROS_MASTER_URI```: URI for ROS server on master machine
- ```ROS_IP```: Your machine's IP address (when connected to the TP_Link network we'll be using). This is not the IP of your container/virtual machine. For Mac users, this should be under Settings>Network. For Windows users, type ```ipconfig``` in the command prompt terminal to check your IP.

To see the variables, use ```echo``` (e.g. ```echo $ROS_MASTER_URI```). To change the variables, ```export ROS_MASTER_URI=<new_value>```

Since these environment variables have to be set each time you open a new terminal, you can automate this by adding the following lines to your ```~/.bashrc```.
```
export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_IP=<your_ip_address>
```
**Note:** if you load and run the image in the repo, this has already been done to your ```.bashrc``.

## 3. Running your scripts
### 3.1. Basic steps
```
cd ~/<workspace_name>
source devel/setup.bash
cd src/<package_name>/ # or whatever path your main script is in
python3 myScript.py
```

Note: Every time you open a terminal, you have to run ```source <workspace_path>/devel/setup.bash``` in order to import the necessary environment variables to run ROS.

### 3.2. Using the API
The only two functions you'll need are ```Action.Request()``` to request for Pepper to do something, ```Info.Request()``` to request data from Pepper or other modules, or ```Info.Send()``` to send data to other modules.

To get the API, place the **PepperAPI** package folder in the same directory as your script, import it in your script, then run ```PepperAPI.init()``` to initialize.

A sample script is below:
```
import PepperAPI
from PepperAPI import Action, Info

"""
Request for STT, process the input, then request for TTS to play the processed message
"""
def myFunc():
	input, output = None, None
	input = Info.Request("ALAudioDevice", {})
	"""
	process mic input message
	"""
	Action.Request("ALTextToSpeech", {"value": output})

if __name__ == "__main__":
	PepperAPI.init()
	myFunc()
```

**```Action.Request(name, params)```**: Request for Pepper to perform an action \\
Parameters:
- **name** : *String*
  Name of API you want to access (see list [here](http://doc.aldebaran.com/2-5/naoqi/index.html)).
- **params** : *Dict*
  Dictionary of parameters to give, which is dependent on the API you're accessing. We'll create full documentation for this once we finish the API.

**```Info.Request(name, params)```**: Request to receive data from Pepper or from other subteams \\
Parameters:
- **name** : *String*
  Name of API you want to access.
- **params** : *Dict*
  Dictionary of parameters to give, which is dependent on the API you're accessing.
Returns:
- **Value** : *Undef*
  Value will also be dependent on which API you're accessing.
  
**```Info.Send(name, params)```**: Request to send data \\
Parameters:
- **name** : *String*
  Name of API you want to access.
- **params** : *Dict*
  Dictionary of parameters to give, which is dependent on the API you're accessing.
Returns:
- **Value** : *Undef*
  Value will also be dependent on which API you're accessing.

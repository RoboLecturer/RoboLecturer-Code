# ROS - Python Documentation

## 1. Overview
Each subgroup should have an instance of Linux on their machine - both virtual machines and dual-boots are fine. The latest Linux kernel that runs ROS is Ubuntu 20.04 which runs ROS Noetic. Noetic is recommended because it supports Python 3 officially. The default Python version for Ubuntu 20.04 is Python 3.8, but it can support Python 3.10 (see [here](https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/) to install). 

After installing ROS, [set up a catkin workspace](#22-setting-up-catkin-workspace) and place your scripts inside. To ask Pepper to do something, or to receive some information from Pepper, use the API in your own scripts: call ```Action.Request()``` or ```Info.Request()``` . For example, this example script requests for Pepper to say "Hello World".
```
#!/usr/bin/env python3
import PepperAPI
from PepperAPI import Action

if __name__ == "__main__":
	# Initialize the API
	PepperAPI.init() 
	# Request action
	Action.Request("ALTextToSpeech", ["Hello World"])
```

The list of Pepper's APIs can be found [here](http://doc.aldebaran.com/2-5/naoqi/index.html).

## 2. Setup
To install Linux, either 
- set up [WSL](https://techcommunity.microsoft.com/t5/windows-11/how-to-install-the-linux-windows-subsystem-in-windows-11/m-p/2701207) (for Windows)
- create a virtual machine (ISO image for Ubuntu 20.04 can be found [here](https://releases.ubuntu.com/20.04.5/)) or 
- dual-boot into Linux

### 2.1. ROS installation
The detailed instructions can be found in the [wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu). The basic steps are, for ROS Noetic:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-fullecho "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.2. Setting up catkin workspace
The basic steps are:
- Create a workspace folder
- Create a ```src/``` folder in the workspace
- Run ```catkin_make``` while in the workspace folder
- Run ```source devel/setup.bash``` while in the workspace folder
```
mkdir -p <workspace_name>/src
cd <workspace_name>
catkin_make

# After catkin_make has finished
source devel/setup.bash
```

Now you can add your python scripts. To do so, 
- Create a package folder in ```<workspace_name>/src```
- Add a folder ```scripts/``` in your package folder
- Add your python scripts, as well as the ```PepperAPI.py``` file in the ```scripts/``` folder
```
# While in your workspace
mkdir -p src/<package_name>/scripts
cd src/<package_name>/scripts
cp /path/to/PepperAPI.py ./
cp /path/to/myScript.py ./
```

The final directory listing should be:
```
<workspace_name>
|    +----build
|    +----devel
|    |         setup.bash
|    \----src
|    |         CMakeList.txt
|    |    \----<package name>
|    |    |    \----scripts
|    |    |              PepperAPI.py
|    |    |              myScript.py
```

Once the scripts have been added, ```cd``` back to your workspace folder and call ```catkin_make``` again. Then, run ```source devel/setup.bash``` to update the environment variables. 

### 2.3. Setting the correct ROS variables
There are two variable you need to ensure are correct in order to connect to the master using ROS: 
- ```ROS_MASTER_URI```: URI for ROS server on master machine
- ```ROS_IP```: Your machine's IP address

To see the variables, use ```echo```. E.g. ```echo $ROS_MASTER_URI```.
To change the variables, type ```export ROS_MASTER_URI=<new_value>```

Since these environment variables have to be set each time you open a new terminal, you can automate this by adding the following to your ```~/.bashrc```.
```
export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_IP=$(ifconfig | grep "inet " | awk '{print $2}') # automatically sets IP, but can be manually set with "export ROS_IP=<your_ip_address>"
```

## 3. Running your scripts
### 3.1. Basic steps
1. ```cd``` to your workspace folder.
2. Run ```source devel/setup.bash```.
3. Ensure your python main script has ```#!/usr/bin/env python``` at the top.
4. Make your python main script executable with ```chmod +x myScript.py```.
5. Ensure your ```ROS_MASTER_URI``` and ```ROS_IP``` variables have been set correctly (see [here]() for details)
6. Run your script with ```rosrun <package_name> myScript.py```.

Note: Every time you open a terminal, you have to run ```source <workspace_path>/devel/setup.bash``` in order to import the necessary environment variables to run ROS.

### 3.2. Using the API
The only two functions you'll need are ```Action.Request()``` to request for Pepper to do something, or ```Info.Request()``` to request for some information from Pepper.

To get the API, place the ```PepperAPI.py``` script in the same folder as your script, then import it, and run ```PepperAPI.init()``` to initialize.

A sample script is below:
```
import PepperAPI
from PepperAPI import Action, Info

"""
Request for STT, process the input, then request for TTS to play the processed message
"""
def myFunc():
	input, output = None, None
	input = Info.Request("ALAudioDevice", [])
	# process mic input message
	Action.Request("ALTextToSpeech", [output])

if __name__ == "__main__":
	PepperAPI.init()
	myFunc()
```

**```Action.Request(name, params)```**: Request for Pepper to perform an action
Parameters:
- **name** : *String*
  Name of API you want to access (see list [here](http://doc.aldebaran.com/2-5/naoqi/index.html)).
- **params** : *List*
  List of parameters to give, which is dependent on the API you're accessing. We'll create full documentation for this once we finish the API.

**```Info.Request(name, params)```**: Request to receive data from Pepper or from other subteams
Parameters:
- **name** : *String*
  Name of API you want to access.
- **params** : *List*
  List of parameters to give, which is dependent on the API you're accessing.
Returns:
- **Value** : *Undef*
  Value will also be dependent on which API you're accessing.
  
**```Info.Send(name, params)```**: Request to send data (for other subteams to access)
Parameters:
- **name** : *String*
  Name of API you want to access.
- **params** : *List*
  List of parameters to give, which is dependent on the API you're accessing.
Returns:
- **Value** : *Undef*
  Value will also be dependent on which API you're accessing.

## 4. FAQs

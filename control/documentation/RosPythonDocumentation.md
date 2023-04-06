# ROS - Python Documentation
## Contents
1. [Overview](#1-overview)
2. [ROS Setup](#2-ros-setup)
   1. [Linux and ROS installation](#21-linux-and-ros-installation)
   2. [Catkin workspace](#22-catkin-workspace)
   3. [Setting ROS variables](#23-setting-ros-variables)
   4. [Running your scripts](#24-running-your-scripts)
3. [PepperAPI](#3-pepperapi)
4. [Testing](#4-testing)
5. [FAQs](#5-faqs)

## 1. Overview
Install your version of ROS. ROS Noetic (Ubuntu 20.04) is recommended since it supports Python 3 officially. The default Python version for Ubuntu 20.04 is Python 3.8, but it should support Python 3.10 (see [here](https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/) to install). For Docker users, a sample image is hosted [here](https://imperiallondon-my.sharepoint.com/:f:/g/personal/rcc22_ic_ac_uk/ErFCcfyKCCNFlZ81R5T2wsMBZ_YBO-EgddnCDjM6Fsgfiw?e=irSsJh). It's basically the ros-noetic-desktop-full image with some helpful tools installed and a sample workspace and package.

After installing ROS, [set up a workspace](#22-setting-up-your-workspace) and place your scripts inside. To ask Pepper to do something, or to receive some information from Pepper, use the API in your own scripts: call ```Action.Request()```,```Info.Request()``` or ```Info.Send()```. For example, this script requests for Pepper to say "Hello World".
```
#!/usr/bin/env python3
import PepperAPI
from PepperAPI import Action

if __name__ == "__main__":	
	PepperAPI.init("speech_node")                              # initialize the API	
	Action.Request("ALTextToSpeech", {"value":"Hello World"})  # request action
```

The list of Pepper's APIs can be found [here](http://doc.aldebaran.com/2-5/naoqi/index.html). API names to send/receive data between the subteams can be defined by you (e.g. RLContent).

## 2. ROS Setup
### 2.1. Linux and ROS installation
See **[LinuxSetup.md](https://github.com/RoboLecturer/RoboLecturer-Code/tree/main/control/documentation/LinuxSetup.md)** for the full details.

### 2.2. Catkin workspace
**Note:** if you load and run the [provided image](https://imperiallondon-my.sharepoint.com/:f:/g/personal/rcc22_ic_ac_uk/ErFCcfyKCCNFlZ81R5T2wsMBZ_YBO-EgddnCDjM6Fsgfiw?e=irSsJh), a sample workspace and package has been created for you.

This section is needed to convert the ROS messages needed into Python packages that you can import in your scripts (e.g. ```std_msgs.msg.String```, ```api.msg```). Otherwise, you'll probably get the warning **```No module found: XXX```**. The basic steps are:
```
mkdir -p ~/<workspace_name>/src         # create workspace folder with "src" folder inside
cd <workspace_name>/src
cp -r ~/RoboLecturer-Code/api ./	# copy the "api" folder in the repo into the "src" folder
cd ..                                   # return to workspace folder
catkin_make                             # run catkin_make
source devel/setup.bash                 # source the environment variables
```
The directory listing should be:
```
<workspace_name>
|    +----build
|    +----devel
|    |         setup.bash
|    \----src
|    |    +----api
```
The workspace can be created anywhere. For convenience, these instructions create it in your **/home/$USER/** folder (**~/**).

Everytime you open a new terminal, be sure to run ```source ~/<workspace_name>/devel/setup.bash```.

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
**Note:** if you load and run the [provided image](https://imperiallondon-my.sharepoint.com/:f:/g/personal/rcc22_ic_ac_uk/ErFCcfyKCCNFlZ81R5T2wsMBZ_YBO-EgddnCDjM6Fsgfiw?e=irSsJh), this has already been done to your ```.bashrc```.

### 2.4. Running your scripts
1. Ensure that there's an updated version of the **PepperAPI** package folder in the same directory as your main script.
2. You should've already [built your catkin workspace](#22-catkin-workspace) somewhere. Source the variables with ```source ~/<workspace_name>/devel/setup.bash```.
3. Navigate to the folder with your main script and run ```python myScript.py```.

## 3. PepperAPI
For the full documentation, refer to **[PepperAPI.md](https://github.com/RoboLecturer/RoboLecturer-Code/tree/main/PepperAPI/README.md)**.

**Note**: This API is only for sending/receiving info between Pepper and your module, or between your module and other modules. For CV, Web and Speech modules that require communication with your camera, mic or web browser, please set that up individually.
  
## 4. Testing
During testing, remember to do the following tasks before starting:
- Network
  1. Turn off Buster's Phone WIFI and enable hotspot.
  2. Re-set MASTER IP to the correct address - ```export ROS_MASTER_URI=http://192.168.0.XXX:11311```
  3. Re-set MODULE IP to the correct address - ```export ROS_IP=<your_ip>```
  4. Connect to TP-Link
  5. Check connection through TP-Link - ```ping google.com```
- Pull the latest version of the Repo (if you are using Docker).
- Run ```source .bashrc``` and ```source ~/catkin_ws/devel/setup.bash```.

## 5. FAQs
- **ERROR: Unable to communicate with master!** when running ROS commands
  1. Is a ```roscore``` running?
  2. If the line ```source /opt/ros/noetic/setup.bash``` is not in your ```~/.bashrc```, run it, then add it to your ```~/.bashrc``` so you can avoid having to run it everytime you open a terminal.
  3. Run ```source ~/<workspace_name>/devel/setup.bash```.
  4. Your ```ROS_MASTER_URI``` may be different from that of the Master running roscore.
  
- **ERROR: Unable to start XML-RPC server, port 45100 is already in use**. 

  If you've called ```PepperAPI.init()``` in multiple scripts, check that the module name you provided as the argument remains the same across all scripts.

- **```No module found: api```**

  You might not have built the api package (see [Section 2.2](#22-catkin-workspace)). If you have, make sure you've run ```source devel/setup.bash```.

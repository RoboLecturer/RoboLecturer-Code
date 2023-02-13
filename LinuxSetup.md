# Linux Setup For ROS
There're 3 options depending on your OS: 
1. Docker for Mac
2. WSL (Windows Subsystem Linux) for Windows, or 
3. VirtualBox (for both OSes).

## Contents
1. [Windows Subsystem Linux (WSL)](#1-windows-subsystem-linux-wsl)
2. [Docker for MacOS](#2-docker-for-macos)
3. [VirtualBox](#3-virtualbox)

## 1. Windows Subsystem Linux (WSL)
### 1.1. Install WSL
1. Install WSL by running this command in Windows Powershell (administrator):
```
wsl --install
```
2. Install Ubuntu 20.04 from the app store and open it.
3. Start Ubuntu 20.04, then downgrade the WSL version to WSL1 by running this command in Windows Powershell (administrator):
```
# after starting Ubuntu at least once
wsl --set-version Ubuntu-20.04 1
```

### 1.2. Install ROS
You can install ROS manually or install Docker then pull the Docker image for ROS Noetic.

If you want to install ROS manually:
The detailed instructions can be found in the [wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu). The basic steps are, for ROS Noetic:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you want to work with the Docker container, you have to install Docker on both your Windows machine and WSL.
1. For Docker Desktop for Windows, under Settings>General, check "Expose daemon on tcp://localhost:2375 without TLS"
2. To install Docker in WSL, follow the instructions in [section 3.2](#32-install-ros).

# 2. Docker for MacOS
1. Install Docker Dekstop for MacOS
2. Your docker image must be run with port forwarding for the range 45100 to 45200. This range allows for each machine to have up to 50 different scripts that can call the API, as each script must have a new node, and a node cannot be initialised with the same ports as another. An example command is:
```
docker run --name=<container-name> -p 45100-45200:45100-45200 -it ros-dev
```
3. Since you'll be using our defined API, you don't need to worry about the ROS wrapping part. But if you just want to test connectio with the master, your publish node must be initiated with the **xmlrpc_port** and **tcpros_port** within the 45100~45200 range. A sample script ```publisher.py``` is:
```
import rospy
from std_msgs.msg import String

if __name__ == "__main__:
  rospy.init_node("my_node", xmlrpc_port=45100, tcpros_port=45101)
  pub = rospy.Publisher("my_topic", String, queue_size=10)
  while not rospy.is_shutdown():
    pub.publish("Hello World")
    rospy.Rate(1).sleep()
```
On a separate master/slave machine, the command ```rostopic echo /my_topic``` can be used to verify communication between nodes is working.

## 3. VirtualBox
### 3.1. Install VirtualBox
1. Install the latest VirtualBox (with Guest Additions) and the [Ubuntu 20.04 desktop image](https://releases.ubuntu.com/20.04.5/).
2. Create the virtual machine and install Ubuntu (see [here](https://www.ktexperts.com/how-to-install-ubuntu-20-04-1-lts-on-windows-using-virtualbox/)).
3. For connecting to the Master, you need to enable "Bridged Adapter" under "Network" settings.

### 3.2. Install ROS
As with WSL, you can either install ROS manually or install Docker and pull the Docker image. This section describes how to do the latter.

1. Install Docker
```
sudo apt-get update -y
sudo apt-get install -y apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update -y
sudo apt-get install -y docker-ce
sudo usermod -aG docker $USER
newgrp docker
```
2. Add ```$HOME/.local/bin``` to path (if not already added) with
```
export PATH="$PATH:$HOME/.local/bin
```
3. Download the image [here](https://imperiallondon-my.sharepoint.com/:f:/g/personal/rcc22_ic_ac_uk/ErFCcfyKCCNFlZ81R5T2wsMBZ_YBO-EgddnCDjM6Fsgfiw?e=irSsJh), extract the **ros-dev.tar** image from the gzip file, load it, then run in a container.
```
cd /mnt/c/Users/<YOURUSERNAME>/Downloads/
tar -xzvf ros-dev.tar.gz
docker load -i ros-dev.tar
docker run --name=<container-name> --net=host -it ros-dev
```
The above command is only for the first run. For subsequent runs, the created container can be accessed with
```
docker start <container-name> # just run once
docker exec -it <container-name> bash # run for each terminal you open
```
4. If you want to install more packages, update the container
```
apt-get -y update
```

**Useful commands**
- Copy files from your laptop to the docker image:
```
docker cp [-r] SRC_PATH <container-name>:DEST_PATH
```
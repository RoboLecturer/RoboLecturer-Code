# Linux Setup For ROSa
You'll need a Linux machine, even if you choose to initialize your environment with Docker. [Windows Subsystem Linux (WSL)](#2-windows-subsystem-linux-wsl) is easier to setup but only gives you a terminal interface. [VirtualBox](#1-virtualbox) (or other VMs) gives you a GUI for easier navigation, but requires the installation of the desktop image and may be much slower if your computer isn't great.

Since we basically only need ROS, we can just grab the docker image for ROS Noetic. But honestly, it was much easier to just install ROS Noetic than installing Docker, especially if you're using WSL.

## Contents
1. [VirtualBox](#1-virtualbox)
- [Install VirtualBox](#11-install-virtualbox)
- [Install ROS with Docker](#12-install-ros-with-docker)
- [Install ROS manually](#13-install-ros-manually)
2. [Windows Subsystem Linux (WSL)](#2-windows-subsystem-linux-wsl)
- [Install WSL](#11-install-wsl)
- [Install ROS with Docker for WSL](#12-install-ros-with-docker-for-wsl)
- [Installing ROS manually for WSL](#13-install-ros-manually-for-wsl)

## 1. VirtualBox
### 1.1. Install VirtualBox
1. Install the latest VirtualBox (with Guest Additions) and the [Ubuntu 20.04 desktop image](https://releases.ubuntu.com/20.04.5/).
2. Create the virtual machine and install Ubuntu (see [here](https://www.ktexperts.com/how-to-install-ubuntu-20-04-1-lts-on-windows-using-virtualbox/)).

### 1.2. Install ROS with Docker
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
```
2. Add ```$HOME/.local/bin``` to path (if not already added) with
```
export PATH="$PATH:$HOME/.local/bin
```
3. Open a new terminal and check that docker is running with
```
docker info
```
4. Pull and run the ROS image with
```
docker pull osrf/ros:noetic-desktop-full
docker run --name=ros-dev [--net=host] -it osrf/ros:noetic-desktop-full bash
```
The above command is only for the first run. For subsequent runs, the created container can be accessed with
```
docker start ros-dev # just run once
docker exec -it ros-dev bash # run for each terminal you open
```
5. Source ROS
```
echo source "/opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
6. Update and install essential packages
```
apt-get -y update
apt install vim net-tools
```

**Useful commands**
- Copy files from your laptop to the docker image:
```
docker cp [-r] SRC_PATH ros-dev:DEST_PATH
```

### 1.3. Install ROS manually
The detailed instructions can be found in the [wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu). The basic steps are, for ROS Noetic:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-fullecho "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Windows Subsystem Linux (WSL)
### 2.1. Installing WSL
1. Install WSL by running this command in Windows Powershell (administrator):
```
wsl --install
```
2. Install Ubuntu 20.04 from the app store and open it.
3. Downgrade the WSL version to WSL1 by running this command in Windows Powershell (administrator):
```
# after starting Ubuntu at least once
wsl --set-version Ubuntu-20.04 1
```

### 2.2. Installing ROS with Docker for WSL
1. Install Docker Desktop for Windows
2. Launch Docker Desktop. Open Settings, and under General, check "Expose daemon on tcp://localhost:2375 without TLS"
3. Launch Ubuntu 20.04 and install Docker as per the instructions for [Docker installation for VirtualBox](#12-install-ros-with-docker)

### 2.3. Installing ROS manually for WSL
Same as the [ROS installation for VirtualBox](#12-install-ros-manually).

# Steps for adding USB device support:

## MacOS:

### Prerequisites

1. Download UTM [here](https://mac.getutm.app/).

2. Download Ubuntu20 image [here](http://cdimage.ubuntu.com/focal/daily-live/current/focal-desktop-arm64.iso). **note: if you're not using an ARM-based mac, then you'll need a slightly different image.

### Set Up Ubuntu machine

1. Open UTM and click on "Create a New Virtual Machine".

2. Choose "Virtualize", then "Linux".

3. Under the "Boot ISO Image" section, browse to find the Ubuntu20 image downloaded earlier.

4. You can change the "Hardware", "Storage", and "Shared Directory" menus as needed. Leaving them at default should also work (this can be adjusted later).

5. On the "Summary" page, click save.

6. Click the three lines icon on the very top right to edit settings for the VM you just created.

7. Click "Input" from the left menu, then tick the box for "Share USB devices from host". Save the settings.

8. Click the play button to start the VM and proceed through the steps for installing Ubuntu. If you see a firmware bug, wait a few minutes to see if Ubuntu still boots (usually it will). All the default settings during the Ubuntu setup should suffice.

9. When the installation completes, **don't** click "Restart Now". Instead, close the popup and X out of the Ubuntu image. Click "OK" on the popup that asks if you are sure you want to exit.

10. Back on the UTM main window, scroll to the bottom of the settings of the virtual machine you just created. Click the drop down menu that says "CD/DVD", and hit "clear".

11. Now if you restart your virtual machine by clicking the play button, it should open up to the ubuntu log in screen where you can log in with the username/password you created earlier.

### Workspace Set Up

1. Open a terminal by right-clicking on the desktop and choosing "Open in Terminal".

2. Here, to set up ROS, you can follow LinuxSetup.md section 1.2 [here](https://github.com/RoboLecturer/RoboLecturer-Code/blob/api/LinuxSetup.md#:~:text=Ubuntu%2D20.04%201-,1.2.%20Install%20ROS,-You%20can%20install). Commands are also listed below for convenience:

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt install curl
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        sudo apt update
        sudo apt install ros-noetic-desktop-full
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc

3. If your USB device/script uses gstreamer, you will also have to run:

        sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

4. Plug your USB device into your machine. On the very top menu bar on the virtual machine window, you should see a usb icon (fourth icon from the right hopefully). Click on the icon and select the USB device you wish to use. Click "Allow" on any prompts that pop up.

5. Now if you run the following commands, you should see your device listed:

        sudo apt-get install v4l-utils
        v4l2-ctl --list-devices

6. Follow the rest of the workspace setup in README.md section 2.2 [here](https://github.com/RoboLecturer/RoboLecturer-Code/tree/api#21-linux-and-ros-installation:~:text=2.2.%20Setting%20up%20your%20workspace).
    * For CV, [here](https://github.com/RoboLecturer/RoboLecturer-Code/blob/cv/CV_workspace_setup.md) are further instructions on using a pre-setup workspace.

## Windows: will be added later.

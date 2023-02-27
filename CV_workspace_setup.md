# CV Workspace Set-up

1. In the terminal where you wish to set up your workspace, run:
        
        git clone -b cv_workspace https://github.com/RoboLecturer/RoboLecturer-Code.git

2. Move the workspace to the desired location:

        cp -r ./RoboLecturer-Code/cv_workspace/ <path to desired workspace location>
    
3. Each time you make a code/file change in the workspace, run the following commands:

        cd <path to workspace>/cv_workspace
        catkin_make
        source devel/setup.bash

4. Then, run main.py:

        python3 <path to main.py>
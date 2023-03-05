# Setting up Robot Web Server

This docker image makes use of the ROS Melodic distrbution in a Ubuntu 18.0.4 environment.

* `docker build -t ros_server .`
* `docker run -it -p 45100:45100 -p 45101:45101 -p 9000:9090 --add-host="host.docker.internal:192.168.0.100" ros_server`  (-it opens interactive shell by default)
* Run the following commands in the directory `~/catkins_ws` of the docker container shell:
    ```bash
    catkin build
    source devel/setup.bash
    ``` 
* Launch webserver using: (& forces tasks to run in the background)
    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch _port:=9090 websocket_external_port:=9000 --screen &
    ``` 
* Run ROS core in the background using the command `roscore` &

Once running, we can publish and listen to topics as follows:

* Change slide: `rostopic pub /change_slide std_msgs/String "data: 'increment|0'" -1`

* Trigger quiz: `rostopic pub /trigger_quiz std_msgs/String "data: 'increment|0'" -1`

* Listen and display receieved messages: `rostopic echo /txt_msg` where /txt_msg is the topic name.

## Sequencce of startup

1. Start docker container 
2. Catkin build
3. source devel/setup.bash
4. export ROS_MASTER_URI=http://192.168.0.102:11311
5. Launch Web socket
6. export ROS_IP to allow for publishing //setting this prevents from local comms


## Resources

Some useful resources:
* https://catkin-tools.readthedocs.io/en/stable/installing.html
* https://msadowski.github.io/ros-web-tutorial-pt1/
* http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge
* http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
* http://robotwebtools.org/

Note:
* Topics are intended for unidirectional, streaming communication. Nodes that need to perform remote procedure calls, i.e. receive a response to a request, should use services instead.





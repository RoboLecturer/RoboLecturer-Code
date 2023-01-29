## Setting up Robot Web Server

This docker image makes use of the ROS Melodic distrbution in a Ubuntu 18.0.4 environment.

* `docker build -t ros_server .`
* `docker run -p 9000:9090 -it ros_server `  (-it opens interactive shell by default)
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

* Publish: `rostopic pub /txt_msg std_msgs/String "data: 'Test message'" -1`, where /txt_msg is the name of the topic and std_msgs/String is the message format.

* Listen and display receieved messages: `rostopic echo /txt_msg` where /txt_msg is the topic name.

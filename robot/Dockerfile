#Select suitable ROS distribution here - packaged inside Ubuntu container
FROM ros:melodic-robot
#FROM ros:noetic-ros-base-focal


#Update and install pip3
RUN sudo apt-get update
RUN sudo apt-get install python3-pip --assume-yes
RUN sudo apt-get -y install nano

#Install catkin tools
RUN git clone https://github.com/catkin/catkin_tools.git
WORKDIR /catkin_tools
RUN sudo pip3 install -r requirements.txt --upgrade
RUN sudo python3 setup.py install --record install_manifest.txt



#Create catkins workspace and install ROS dependencies
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN catkin create pkg robot_gui_bridge --catkin-deps rosbridge_server
WORKDIR /root/catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y

#Adding webserver launch file and running
RUN mkdir /root/catkin_ws/src/robot_gui_bridge/launch
COPY ./websocket.launch /root/catkin_ws/src/robot_gui_bridge/launch
COPY ./scripts/publish.py /root/catkin_ws/src/robot_gui_bridge/publish.py
COPY ./scripts/forwarder.py /root/catkin_ws/src/robot_gui_bridge/forwarder.py
RUN export ROS_MASTER_URI=http://192.168.0.102:11311
RUN export ROS_IP=192.168.0.100
# RUN . /opt/ros/melodic/setup.sh && \
#     catkin build
# RUN source devel/setup.bash

#Entrypoint
#docker run -p 9000:9090 -it ros_server
#roslaunch rosbridge_server rosbridge_websocket.launch _port:=9090 websocket_external_port:=9000 --screen &
#roscore & //runs roscore master in the background
#export ROS_MASTER_URI=http://192.168.0.101:11311


EXPOSE 9000
#ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


#ip=$(hostname -I)
# export ROS_IP=$ip
# export ROS_MASTER_URI=http://172.17.0.2:11311
# echo $ROS_IP
# echo $ROS_MASTER_URI
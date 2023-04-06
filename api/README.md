This is a ROS package that contains the **CVInfo** ROS message definition, which contains bounding box information about the detected raised hand, the frame resolution and confidence score, which the Compute Vision module sends to the Control module.

The **CVInfo** message contains the following information:
- ```x``` : *float32*  top-left x-coordinate of detected hand in frame
- ```y``` : *float32* top-right y-coordinate of detected hand in frame
- ```w``` : *float32* width of hand in pixels
- ```h``` : *float32* height of hand in pixels
- ```frame_width``` : *uint16* width of frame
- ```frame_height``` : *uint16* height of frame
- ```score``` : *float32* confidence score of model for detected hand

The package is sourced by placing in a catkin workspace and sourcing that workspace. Detailed instructions are given [here](https://github.com/RoboLecturer/RoboLecturer-Code/blob/main/control/documentation/RosPythonDocumentation.md#22-catkin-workspace).

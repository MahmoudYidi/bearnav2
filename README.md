# Navigros2 (Bearnav-ROS2)
## Overview

ROS2 Version of the code base [a bearnav](https://github.com/LCAS/bearnav2)
## Installation

Clone the repository into a ROS workspace and build.

```bash
git clone https://github.com/MahmoudYidi/navigros2.git
colcon build --packages-select navigros2
```

## Simple GUI Usage
The new update allows you to simply control your mapping and repeating from a GUI based control dashboard.
Simply launch the node:
```bash
ros2 launch navigros2 navigros2_launch.py
```
Navigate to the package directory and then run the GUI script:
```bash
python3 map_gui.py
```
You should get the display below
\
![image](https://github.com/user-attachments/assets/b5106119-949f-49ec-9465-f7faaff218d6)
\
\
Simply type in the name of the map and use as follows:\
:arrow_forward: Click Start Mapping and drive/teach your robot\
:arrow_forward: Click Stop Mapping when done to save session.\
:arrow_forward: Relaunch your node and GUI, type in the saved map and click repeat to repeat the map.

If you want to have GUI-related output topics for visualisation (runs slower), then also launch:
```bash
ros2 launch navigros2 navigros2-gui-launch.py
```

## Classic Usage
The usage is slightly different with the ROS 1 version. 

As mentioned Initially, you can run the package with and without GUI-related output topics.
For a simple launch without GUI-topics for visualisation:

```bash
ros2 launch navigros2 navigros2_launch.py
```
This simply just launch the ***navigros2_launch*** . Faster as no additional computations are done. However if you require the gui, may run slower, but provides visualisation. You need to also launch the GUI using:

```bash
ros2 launch navigros2 navigros2-gui-launch.py
```




### Mapping
To start mapping simply navigate to the package directory and  run:
```bash
./map_control.sh start_map name
```
Where '*name*' is the the name of the map. After succesfully running this, you can simply drive the robot around to your desired goal. if you are done, run:

```bash
./map_control.sh stop_map name
```

to save the map and end the mapping session. 

To repeat your map, simply run:
```bash
./map_control.sh repeat_map name
```
\
:bangbang: Don't forget to map your topics in the launch files according to your robot topics.

# Migration
# Overview

ROS2 Version of the code base _______
## Installation

Clone the repository into a ROS workspace and build.

```bash
git clone _________
cd ___________
colcon build
```

## Usage
The usage is slightly different with the ROS 1 version. 

If you want to run the system with gui, then:

```bash
ros2 launch navigros2 navigros2_launch.py
```
This simply just launch the ***navigros2_launch*** . Faster as no additional computations are done. However if you require the gui, may run slower, but provides more feedback using gui interface:

```bash
ros2 launch navigros2 navigros2-gui-launch.py
```

### Mapping

A bash file has been created to simplify the mapping process. 

To start mapping simply run:
```bash
./map_control.sh start_map name
```
Where *name* is the the name of the map. After succesfully running this, you can simply drive the robot around to your desired goal. if you are done, run:

```bash
./map_control.sh stop_map name
```

to save the map and end the mapping session. 

To repeat your map, simply run:
```bash
./map_control.sh repeat_map name
```
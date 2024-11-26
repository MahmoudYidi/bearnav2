# Navigros2 (Bearnav-ROS2)
## Overview

ROS2 Version of the code base [a link](https://github.com/LCAS/bearnav2)
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
This simply just launch the ***navigros2_launch*** . Faster as no additional computations are done. However if you require the gui, may run slower, but provides more feedback using gui interface, You need to also launch the GUI using:

```bash
ros2 launch navigros2 navigros2-gui-launch.py
```

### Mapping

A bash file has been created to simplify the mapping process. Before that simply copy the .sh file to your workspace (do this once).
```bash
cd src/navigros2/
cp map_control.sh ../../
cd ../../
```

To start mapping simply run:
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

Don't forget to map your topics in the launch file to your desired robot topics.

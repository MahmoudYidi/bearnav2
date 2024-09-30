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
ros2 launch navigros2 navigros2-gui-launch.py
```

and then followed by: 

```bash
ros2 launch navigros2 navigros2_launch.py
```
However, may run slower, but provides more feedback using gui interface. Otherwise you can simple just launch the navigros2_launch if these aren't required. Faster as no additional computations are done.


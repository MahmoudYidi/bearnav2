# Navigros2 (Bearnav-ROS2)

## Overview

**Navigros2** is the ROS2 version of the [Bearnav project](https://github.com/LCAS/bearnav2). It provides a framework for autonomous robot mapping and navigation. This project includes functionalities for recording, saving, and repeating robot maps, alongside optional graphical interfaces for control.

## Installation

To set up **Navigros2** in your ROS workspace, follow these steps:

1. Clone the repository into your workspace:
    ```bash
    git clone https://github.com/MahmoudYidi/navigros2.git
    cd navigros2
    ```

2. Install required Python dependencies:
    ```bash
    pip install -r requirements.txt
    ```

3. Build the package:
    ```bash
    cd ..
    colcon build --packages-select navigros2
    ```

## Usage

### Simple GUI Usage

The new version of **Navigros2** comes with a simple GUI for controlling mapping and repeating actions. To use the GUI:

1. Launch the ROS2 nodes:
    ```bash
    ros2 launch navigros2 navigros2_launch.py
    ```

2. Navigate to the package directory and run the GUI script:
    ```bash
    python3 map_gui.py
    ```

    This will open the GUI interface.

    ![image](https://github.com/user-attachments/assets/b5106119-949f-49ec-9465-f7faaff218d6)

3. In the GUI, enter the name of the map and use the following controls:
    - **Start Mapping**: Click to begin the mapping process and move the robot.
    - **Stop Mapping**: Click to save the map once you’re done.
    - **Repeat Map**: Relaunch the node and GUI, enter the saved map name, and click **Repeat** to replay the map.

For visualization of GUI-related topics (this may impact performance), launch the following:
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
map_control.sh file has been created to directly trigger the mapping and reapeting actions:
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

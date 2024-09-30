# Migration
# Overview

This ROS package allows a robot to be taught a path through an environment using a camera. The robot can then retrace this path at a future point in time, correcting any errors. The theory can be found in the linked paper. The system works by replaying the robot commands from during the training phase and applying slight adjustments to them according to how the camera looks compared to during the teaching phase. As long as the error is reasonable, the robot will converge to the original path.

# Installation

Clone the repository into a ROS workspace and build.

```bash
git clone <repository_url>
cd <workspace_directory>
catkin_make
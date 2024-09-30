cmake_minimum_required(VERSION 3.8)
project(navigros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alignment.msg"
  "msg/FeatureLocations.msg"
  "msg/FloatList.msg"
  "srv/SetDist.srv"
  "srv/SetClockGain.srv"
  "action/MapMaker.action"
  "action/MapRepeater.action"
  DEPENDENCIES
    std_msgs
    action_msgs # Add packages that above messages depend on,
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install Python modules
install(DIRECTORY
  src/alignment/
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  src/distance/distance_ros.py
  src/distance/distance.py
  src/gui/histogram_viz.py
  src/gui/live_viz.py
  src/gui/matches_viz.py
  src/master/mapmaker_ros.py
  src/master/repeater_ros.py
  src/navigator/navigator.py
  src/navigator/navigator_ros.py
  src/preprocess/preprocess_ros.py
  src/alignment/histogram.py
  src/alignment/alignment.py
  src/alignment/alignment_ros.py



  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

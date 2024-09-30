
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (Change to match your robot)
        DeclareLaunchArgument('camera_topic', default_value='/front_camera/image_raw', description='Camera topic name'),
        DeclareLaunchArgument('feature_type', default_value='2', description='Feature type index (example: 0 for SIFT, 1 for SURF in [SIFT, SURF, KAZE, AKAZE, BRISK, ORB], Note, SURF not avail'),
        #######################################################################################################

        ########### NO NEED TO CHANGE BELOW  ###########

        # Live Viz node
        Node(
            package='navigros2',
            executable='live_viz.py',
            name='live_viz',
            output='screen',
            respawn=True,
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'feature_type': LaunchConfiguration('feature_type')  
            }],
            namespace='navigros2'
        ),

        # Nodes
        Node(
            package='navigros2',
            executable='histogram_viz.py',
            name='histogram_viz',
            output='screen',
            respawn=True,
            parameters=[{}],
            namespace='navigros2'
        ),

        Node(
            package='navigros2',
            executable='matches_viz.py',
            name='matches_viz',
            output='screen',
            respawn=True,
            parameters=[{}],
            namespace='navigros2'
        )


    ])

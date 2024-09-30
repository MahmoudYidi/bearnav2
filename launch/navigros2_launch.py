from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (Current Setup is for AOC devcontainer hunter simulation, This is the only part you should change )
        DeclareLaunchArgument('camera_topic', default_value='/front_camera/image_raw', description='Camera topic name'),
        DeclareLaunchArgument('cmd_vel_pub', default_value='/diff_drive_controller/cmd_vel_unstamped', description='The topic that cmd_vel messages will be published on when replaying a map (can be the same as cmd_vel_sub)'),
        DeclareLaunchArgument('cmd_vel_sub', default_value='/diff_drive_controller/cmd_vel_unstamped', description='The topic that cmd_vel messages will be recorded from when making a map (can be the same as cmd_vel_pub)'),
        DeclareLaunchArgument('odom_topic', default_value='/diff_drive_controller/odom', description='The topic that odometry information will be read from'),
        DeclareLaunchArgument('additional_record_topics', default_value='/sound_play/goal', description='Space separated topics names for additional recording'),
        #DeclareLaunchArgument('alignment_method', default_value='SIAM', description='Alignment method to use'),  
        DeclareLaunchArgument('alignment_method', default_value='SIAM', description='Alignment method to use (Choice [SIAM, SIFT, KAZE, AKAZE, BRISK, ORG, VGG])'),
        ######################################################################################################

        ########   NO NEED TO CHANGE ANYTHING BELOW IF NOT TWEAKING PARAMETERS OF ALGORITHM   ###########
        # Alignment node
        Node(
            package='navigros2',
            executable='alignment_ros.py',
            name='alignment',
            output='screen',
            parameters=[{
                'feature_type': LaunchConfiguration('alignment_method'),
            }],
            namespace='navigros2'
        ),

        # Distance node
        Node(
            package='navigros2',
            executable='distance_ros.py',
            name='distance',
            respawn=True,
            output='screen',
            arguments=['-OO'],
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_sub'),
                'odom_topic': LaunchConfiguration('odom_topic')
            }],
            namespace='navigros2'
        ),

        # Mapmaker node
        Node(
            package='navigros2',
            executable='mapmaker_ros.py',
            name='mapmaker',
            respawn=True,
            arguments=['-OO'],
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_sub'),
                'additional_record_topics': LaunchConfiguration('additional_record_topics')
            }],
            output='screen',
            namespace='navigros2'
        ),

        # Repeater node
        Node(
            package='navigros2',
            executable='repeater_ros.py',
            name='repeater',
            respawn=True,
            arguments=['-OO'],
            parameters=[{
                'camera_topic': LaunchConfiguration('camera_topic')
            }],
            output='screen',
            namespace='navigros2'
        ),

        # Navigator node
        Node(
            package='navigros2',
            executable='navigator_ros.py',
            name='navigator',
            parameters=[
                {'velocity_gain': 1.0},
                {'turn_gain': 0.0005},
                {'use_uncertainty': True},
                {'cmd_vel_topic': LaunchConfiguration('cmd_vel_sub')}
            ],
            output='screen',
            namespace='navigros2'
        ),

        # Preprocessor node
        Node(
            package='navigros2',  
            executable='preprocess_ros.py',    
            name='preprocessor',
            output='screen',
            parameters=[{
                'hist_equal': False 
            }],
            namespace='navigros2'
        ),
    ])

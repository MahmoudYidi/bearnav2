#!/usr/bin/env python3

import time
import os
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from navigros2.action import MapRepeater
from navigros2.srv import SetDist, SetClockGain
from navigros2.msg import Alignment
import rosbag2_py
import threading
import numpy as np
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

class ActionServerNode(Node):

    def __init__(self):
        super().__init__('replayer_server')

        # Initialize variables
        self.br = CvBridge()
        self.img = None
        self.map_name = ""
        self.map_step = None
        self.next_step = 0
        self.bag_reader = None
        self.is_repeating = False
        self.file_list = []
        self.end_position = None
        self.clock_gain = 1.0
        self.lock = threading.Lock()
        self.is_shutting_down = False
        self.clock = Clock()

        # services and action server set up
        self.get_logger().info("Waiting for services to become available...")
        self.distance_reset_srv = self.create_client(SetDist, 'set_dist')
        
        self.set_clock_gain_srv = self.create_service(SetClockGain, 'set_clock_gain', self.set_clock_gain)

        while not self.distance_reset_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        # Subscribers
        self.get_logger().info("Subscribing to distance and camera topics")
        self.distance_sub = self.create_subscription(Float64, 'distance', self.distance_cb, 1)
        
        
        self.declare_parameter('camera_topic', '')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.cam_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, 1)

        # Publishers
        self.al_1_pub = self.create_publisher(Image, 'alignment/inputCurrent', 1)
        self.al_2_pub = self.create_publisher(Image, 'alignment/inputMap', 1)
        self.al_pub = self.create_publisher(Alignment, 'correction_cmd', 0)
        self.joy_pub = self.create_publisher(Twist, 'map_vel', 0)

        # Alignment module subscription
        self.al_sub = self.create_subscription(Alignment, 'alignment/output', self.align_cb, 10) #1000

        # Start the action server
        self.get_logger().info("Starting repeater action server")
        self.action_server = ActionServer(
            self,
            MapRepeater,
            'repeater',
            self.action_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info("Server started, awaiting goal")

    def set_clock_gain(self, request, response):
        self.clock_gain = request.gain
        self.get_logger().info(f'Clock gain set to: {self.clock_gain}')
        #response.success = True
        return response

    def image_cb(self, msg):
        if self.is_repeating:
            #with self.lock:
            self.img = msg  # Update to the latest image
            self.al_1_pub.publish(msg)  # Always publish the latest image
            #self.get_logger().info("Received and published a new image")
            
        

        

    def get_closest_img(self, dist):
        if self.is_shutting_down:
            return
        if len(self.file_list) < 1:
            self.get_logger().warn("No map files found")
            return

        closest_filename = None
        closest_distance = float('inf')
        dist = float(dist)

       
        
        self.file_list = [f.split('.jpg')[0] for f in os.listdir(self.map_name) if f.endswith('.jpg')]
        

        for filename in self.file_list:
            try:
                file_distance = float(filename)  
            except ValueError:
                self.get_logger().warn(f"Filename {filename} is not a valid float")
                continue  #

            diff = abs(file_distance - dist)
            if diff < closest_distance:
                closest_distance = diff
                closest_filename = filename

        if closest_filename:
            fn = os.path.join(self.map_name, f"{closest_filename}.jpg")
            #self.get_logger().info(f"Opening file: {fn}, distance: {dist}")
            img = cv2.imread(fn)

            if img is None:
                self.get_logger().error(f"Failed to load image: {fn}. Please check if the file exists and is readable.")
            else:
                #self.get_logger().info(f" load image: {fn}. ")
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                msg = self.br.cv2_to_imgmsg(img_rgb, encoding='rgb8')
                #msg = self.br.cv2_to_imgmsg(img)
                self.al_2_pub.publish(msg)
        else:
            self.get_logger().warn(f"No closest file found for distance: {dist}")





    def distance_cb(self, msg):
        
        if not self.is_repeating:
                return
            
        if self.is_shutting_down:
            return
        
        if self.img is None:
            self.get_logger().warn("Warning: no image received")
            return
        dist = msg.data
        self.get_closest_img(dist)

        if self.end_position != 0 and dist >= self.end_position:
            self.is_repeating = False
        
        
    

    def align_cb(self, msg):
        self.al_pub.publish(msg)


    def goal_valid(self, goal):
        # Check for valid map directory
        if not goal.map_name:
            self.get_logger().warn("Goal missing map name")
            return False
        
        map_directory = goal.map_name

        if not os.path.isdir(map_directory):
            self.get_logger().warn(f"Can't find map directory: {map_directory}")
            return False

        if not os.path.isfile(os.path.join(map_directory, 'params')):
            self.get_logger().warn(f"Can't find params in directory: {map_directory}")
            return False
        
        # Check for the sub-directory
        sub_directory = os.path.join(map_directory, goal.map_name)
        if not os.path.isdir(sub_directory):
            self.get_logger().warn(f"Can't find sub-directory: {sub_directory}")
            return False

        db_files = [f for f in os.listdir(sub_directory) if f.endswith('.db3')]
        if not db_files:
            self.get_logger().warn(f"Can't find any .db3 files in directory: {sub_directory}")
            return False

        if not os.path.isfile(os.path.join(sub_directory, 'metadata.yaml')):
            self.get_logger().warn(f"Can't find metadata.yaml in directory: {sub_directory}")
            return False

        # Validate positions
        if goal.start_pos < 0:
            self.get_logger().warn("Invalid (negative) start position")
            return False

        if goal.start_pos > goal.end_pos:
            self.get_logger().warn("Start position greater than end position")
            return False

        return True

    #def action_cb(self, goal_handle):
       # self.get_logger().info("New goal received")
        #threading.Thread(target=self.handle_action, args=(goal_handle,)).start()

    def action_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info("New goal received")
        #self.is_repeating = True
        result = MapRepeater.Result()
        

        if not goal_handle.is_active:
            self.get_logger().info("Goal preempted")
            self.shutdown()
            return result
        
        if not self.goal_valid(goal):
            self.get_logger().warn("Ignoring invalid goal")
            result.success = False
            goal_handle.abort()
            return result

        self.img = None 
        self.parse_params(os.path.join(goal.map_name, "params"))

        # Get file list
        #with self.lock:
        all_files = next(os.walk(goal.map_name))[2]

        self.file_list = [filename.split('.')[0] for filename in all_files if ".jpg" in filename]
        self.get_logger().info(f"Found {len(self.file_list)} map files")

        # Set distance to zero
        self.get_logger().info("Resetting distance")
        self.distance_reset_srv.call_async(SetDist.Request(dist=goal.start_pos))
        self.end_position = goal.end_pos
        self.next_step = 0

        self.get_logger().info("Starting repeat")
        self.bag_reader = self.create_bag_reader(goal.map_name)
        self.map_name = goal.map_name
        

        # Create publishers for additional topics
        additional_publishers = {}
        for topic in self.bag_reader.get_all_topics_and_types():
            if topic.name != self.saved_odom_topic:
                msg_type = self.get_message_type(topic.type)
                additional_publishers[topic.name] = self.create_publisher(msg_type, topic.name, 10)
                self.get_logger().info(topic.name)

        # Replay bag
        self.get_logger().info("Starting bag playback")
        
        self.is_repeating = True
        start_time = self.clock.now()

        previous_message_time = None

        for topic, msg, timestamp in self.read_messages_from_bag():
            now = self.clock.now().nanoseconds
            if previous_message_time is None:
                previous_message_time = timestamp
                expected_message_time = now
            else:
                simulated_time_to_go = timestamp - previous_message_time
                corrected_simulated_time_to_go = simulated_time_to_go * self.clock_gain

                error = now - expected_message_time
                sleep_time_nanoseconds = corrected_simulated_time_to_go - error

                # Convert nanoseconds to seconds and nanoseconds
                seconds = int(sleep_time_nanoseconds // 1e9)  
                nanoseconds = int(sleep_time_nanoseconds % 1e9)  
                
                duration = Duration(seconds=seconds, nanoseconds=nanoseconds)
                #clock.sleep_for(corrected_simulated_time_to_go)
                self.clock.sleep_for(duration)
                expected_message_time = now + sleep_time_nanoseconds
                previous_message_time = timestamp

            
            if topic == self.saved_odom_topic:
                self.joy_pub.publish(msg)
            else:
                additional_publishers[topic].publish(msg)

            #if not self.is_repeating or self.get_clock().is_shutdown():
             #   break
            if self.is_repeating == False:
                self.get_logger().info("Stopped")
                break

            while not self.is_repeating:
                if not rclpy.ok():  
                    self.get_logger().info("ROS is shutting down.")
                    break
        self.isRepeating = False
        end_time = self.clock.now()
        duration = end_time - start_time
        self.get_logger().info(f"Rosbag runtime: {duration.nanoseconds / 1e9:.2f} seconds")
        result.success = True
        goal_handle.succeed()
        return result

    def create_bag_reader(self, directory):
        # Expect directory to be the main directory containing sub-directory with .db3 files
        sub_directory = os.path.join(directory, os.path.basename(directory))
        
        if not os.path.isdir(sub_directory):
            raise FileNotFoundError(f"Sub-directory not found: {sub_directory}")

        db_files = [f for f in os.listdir(sub_directory) if f.endswith('.db3')]
        if not db_files:
            raise FileNotFoundError("No .db3 files found in sub-directory")

        db_file = db_files[0]
        file_path = os.path.join(sub_directory, db_file)
        
        storage_options = rosbag2_py.StorageOptions(uri=file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        bag_reader = rosbag2_py.SequentialReader()
        bag_reader.open(storage_options, converter_options)
        return bag_reader


    def read_messages_from_bag(self):
        while self.bag_reader.has_next():
            topic, msg, timestamp = self.bag_reader.read_next()
            yield topic, msg, timestamp

    def get_message_type(self, topic_type):
        # Map topic types to their corresponding ROS message types
        if topic_type == 'sensor_msgs/msg/Image':
            return Image
        elif topic_type == 'geometry_msgs/msg/Twist':
            return Twist
        elif topic_type == 'std_msgs/msg/Float64':
            return Float64
        else:
            raise ValueError(f"Unknown topic type: {topic_type}")

    def parse_params(self, filename):
        with open(filename, "r") as f:
            data = f.read().splitlines()

        for line in data:
            param, value = line.split(":")
            param = param.strip()
            value = value.strip()

            if param == "stepSize":
                self.map_step = float(value)
                self.get_logger().info(f"Step size set to: {self.map_step}")
            elif param == "odomTopic":
                self.saved_odom_topic = value
                self.get_logger().info(f"Saved odometry topic: {self.saved_odom_topic}")

    def check_shutdown(self):
        #if self.action_server.is_preempt_requested():
         #   self.shutdown()
        pass

    def shutdown(self):
        #self.get_logger().warn("Cancelling repeat")
        self.is_repeating = False
        #self.bag_reader.reset()
        if self.bag_reader:
            del self.bag_reader 
        self.is_shutting_down = True
        #self.get_logger().info("Bag reader cleaned up successfully.")
        self.end_position = None
        self.distance_reset_srv.call_async(SetDist.Request(dist=0.0))

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    try:
        node = ActionServerNode()
        
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if hasattr(node, 'shutdown'):
            node.shutdown()
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()

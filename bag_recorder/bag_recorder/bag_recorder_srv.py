from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node


from rclpy.serialization import serialize_message

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import rosbag2_py
from rclpy.time import Time

import os
import datetime
import yaml

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
      
        # QoS profiles subscription definition
        self.map_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_ALL,
          depth=5)

        self.scan_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.camera_image_raw_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.tf_static_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_ALL
          )
        
        self.robot_description_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.global_costmap_costmap_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.local_costmap_costmap_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        self.navigate_to_pose_action = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)


           # Topics and QoS profiles definition
        self.topics_qos_profiles = {
            'initialpose': (PoseWithCovarianceStamped, 'geometry_msgs/msg/PoseWithCovarianceStamped', None),
            'scan': (LaserScan, 'sensor_msgs/msg/LaserScan', self.scan_qos),
            'map': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.map_qos),
            'odom': (Odometry, 'nav_msgs/msg/Odometry', None),
            'cmd_vel': (Twist, 'geometry_msgs/msg/Twist', None),
            'amcl_pose': (PoseWithCovarianceStamped, 'geometry_msgs/msg/PoseWithCovarianceStamped', self.amcl_pose_qos),
            'tf': (TFMessage, 'tf2_msgs/msg/TFMessage', None),
            'tf_static': (TFMessage, 'tf2_msgs/msg/TFMessage', self.tf_static_qos),
            'robot_description': (String, 'std_msgs/msg/String', self.robot_description_qos),
            'global_costmap/costmap': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.global_costmap_costmap_qos),
            'camera/image_raw': (Image, 'sensor_msgs/msg/Image', self.camera_image_raw_qos),
            'local_costmap/costmap': (OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', self.local_costmap_costmap_qos),
            'behavior_tree_log': (BehaviorTreeLog, 'nav2_msgs/msg/BehaviorTreeLog', None),
            'plan': (Path, 'nav_msgs/msg/Path', None),
            'navigate_to_pose/_action/status': (GoalStatusArray, 'action_msgs/msg/GoalStatusArray', self.navigate_to_pose_action)
        }

        self.record = False

         # Writer object
        self.writer = rosbag2_py.SequentialWriter()

        # Service definition
        self.srv = self.create_service(SetBool, 'bag_recorder', self.BagRecorder_callback)
        


    def BagRecorder_callback(self, request, response):

        # URI of the bag to create and the format
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%f")
        #home_dir = os.environ['HOME']
        #path = os.path.join(home_dir, "rosbag_output")
        path = os.path.join(os.getcwd(), "rosbag_output")
        if not os.path.exists(path):
            os.makedirs(path)
        uri = os.path.join(path, "bag_" + timestamp)
        storage_options = rosbag2_py._storage.StorageOptions(uri=uri, storage_id='sqlite3')

        # Default conversion options to store the message in the serialization format the are received in
        converter_options = rosbag2_py._storage.ConverterOptions('', '')

        default_qos = 10
        
        if request.data == True:
            self.record = True
            try:
                self.writer.open(storage_options, converter_options)
            except RuntimeError:
                self.get_logger().error('Failed to create database directory {}'.format(uri))
          
            # Topics' specification and writer's registering process
            for topic, (msg_type, type_string, qos_profile) in self.topics_qos_profiles.items():
                if qos_profile:
                    offered_qos_profiles = self.qos_profile_to_yaml(qos_profile)
                else:
                    offered_qos_profiles = ''
                topic_info = rosbag2_py._storage.TopicMetadata(name=topic, type=type_string, serialization_format='cdr', offered_qos_profiles=offered_qos_profiles)
                self.writer.create_topic(topic_info)

            # Topic's subscription
            for topic, (msg_type, type_string, qos_profile) in self.topics_qos_profiles.items():
                if qos_profile is None:
                    qos_profile = default_qos
                topic_name = topic
                self.subscription = self.create_subscription(msg_type, topic, lambda msg, topic_name=topic: self.topic_callback(msg, topic_name), qos_profile)
        
            
            response.success = True
            response.message = 'Recording...'

        if request.data == False:
            self.record = False            
            if hasattr(self, 'subscription'):
               self.destroy_subscription(self.subscription)

            self.writer = None
            try:
              del self.writer
              self.writer = rosbag2_py.SequentialWriter()
            except NameError:
              print(f"{object_name} exists")

            response.success = False
            response.message = 'Not recording...'
        
        return response

    # Create QoS object from dictionary
    def create_qos_profile(properties):
        qos = QoSProfile()
        for key, value in properties.items():
            setattr(qos, key, value)
        return qos
  
    # Callbacks functions to pass the serialized message to the writer
    def topic_callback(self, msg, topic_name):
        if self.record:
            try:
                self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)
            except RuntimeError:
                self.get_logger().error('{} topic has not been created yet! Call create_topic first.'.format(topic_name))
        

    # Return nanoseconds from a duration
    def duration_to_node(self, duration):
        t = Time(nanoseconds=duration.nanoseconds)
        node = {}
        (node['sec'], node['nsec']) = t.seconds_nanoseconds()
        return node

    # Return yaml output from QoS profile
    def qos_profile_to_yaml(self, qos_profile):
        profile_list = []
        qos = {}
        qos['history'] = int(qos_profile.history)
        qos['depth'] = int(qos_profile.depth)
        qos['reliability'] = int(qos_profile.reliability)
        qos['durability'] = int(qos_profile.durability)
        qos['lifespan'] = self.duration_to_node(qos_profile.lifespan)
        qos['deadline'] = self.duration_to_node(qos_profile.deadline)
        qos['liveliness'] = int(qos_profile.liveliness)
        qos['liveliness_lease_duration'] = self.duration_to_node(qos_profile.liveliness_lease_duration)
        qos['avoid_ros_namespace_conventions'] = qos_profile.avoid_ros_namespace_conventions
        profile_list.append(qos)

        return yaml.dump(profile_list, sort_keys=False)
    
    def __del__(self):
        print("ros2 bag recorder object destroyed")
    
def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

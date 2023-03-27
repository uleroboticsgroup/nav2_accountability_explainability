import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, PoseStamped

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')

        #Load poses from YAML file
        pose_file = os.path.join(get_package_share_directory('goal_sender'), 'config', 'poses.yaml')
        self.poses = self.get_poses_from_yaml(pose_file)        
   
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')


    def get_poses_from_yaml(self, yaml_file):
        with open(yaml_file, 'r') as f:
            poses_yaml = yaml.safe_load(f)['poses']

        if poses_yaml is not None:
            first_pose = poses_yaml[0]['pose']
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position = Point(**first_pose['position'])
            goal_pose.pose.orientation = Quaternion(**first_pose['orientation'])
            
        return goal_pose
      

    def send_goal(self):

        goal_pose = NavigateToPose.Goal()
        goal_pose.pose = self.poses
        
        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(goal_pose, feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info('Result: {0}' + str(result))
        self.get_logger().info('Status: {0}' + str(status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('FEEDBACK:' + str(feedback) )

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node

# import the ExplanationServiceMessage from custom_interfaces package
from custom_interfaces.srv import ExplanationServiceMessage

# import messages used in the subscribed topics
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import math
import numpy as np

class NavigationExplainer(Node):
    DISTANCE_THRESHOLD = 1.2 #(20%)
    OBSTACLE_DISTANCE_THRESHOLD = 1.3 #1.3 m
    def __init__(self):
        super().__init__('navigation_explainer')

        self.previous_distance = float('inf')
        self.previous_goal_id = None
        self.finished_goals = []
        self.changed_route_answer = "I have not changed the planned path"
        self.navigation_status_answer = "No navigation is running"
        self.new_plan = 0

        # QoS profiles subscription definition
        self.scan_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        # Service definition
        self.srv = self.create_service(ExplanationServiceMessage, 'navigation_explainer', self.NavigationExplainer_callback)

        # Topics subscription
        self.subscription_plan = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.subscription_plan

        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, self.scan_qos)
        self.subscription_scan

        self.subscription_nav_status = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)
        self.subscription_nav_status
        

    def NavigationExplainer_callback(self, request, response):        
        if request.question == "Why have you changed the planned path?":           
            response.answer = self.changed_route_answer
        elif request.question == "What is the current navigation status?":
            response.answer = self.navigation_status_answer   
        # return the response parameter
        return response
    
    def plan_callback(self, msg):
        total_distance = 0

        for i in range(len(msg.poses) -1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position
            distance = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
            total_distance += distance
        
        if total_distance > self.previous_distance * NavigationExplainer.DISTANCE_THRESHOLD:
            self.changed_route_answer = "I have changed the planned path"
            if min(self.last_scan.ranges) < NavigationExplainer.OBSTACLE_DISTANCE_THRESHOLD:
                self.changed_route_answer += " because there was an obstacle."
            else:
                self.changed_route_answer += " but I do not know why."
            self.new_plan = 1
        elif self.new_plan == 1:
            self.changed_route_answer += " Then, I followed a new path to the goal pose."
            self.new_plan = 0

        self.previous_distance = total_distance

    def scan_callback(self, msg):
        self.last_scan = msg

    def status_callback(self, msg):
        self.navigation_status_answer = ""
        for status in msg.status_list:
            current_goal_id = status.goal_info.goal_id
            if current_goal_id != self.previous_goal_id and current_goal_id not in self.finished_goals:
                self.previous_goal_id = current_goal_id
                self.navigation_status_answer = "Navigation to a new goal has started."
            if status.status in [2, 4, 5, 6] and current_goal_id not in self.finished_goals:
                status_dict = {
                    2: "is in progress",
                    4: "has succeeded",
                    5: "was cancelled",
                    6: "has aborted"
                }
                self.navigation_status_answer = "Navigation to the goal {}.".format(status_dict[status.status])
                if status.status in [4, 5, 6]:
                    self.finished_goals.append(current_goal_id)
        if not self.navigation_status_answer:
            self.navigation_status_answer = "No navigation is running."



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    ne = NavigationExplainer()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(ne)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
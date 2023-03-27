from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node

from nav2_msgs.msg import BehaviorTreeLog

class BTStatusRecorderClientAsync(Node):

    def __init__(self):

        # Initialize the node
        super().__init__('btstatus_recorder_client')

        # Create the service client object
        self.client = self.create_client(SetBool, 'bag_recorder')

        # Check once per second the service's availability
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Service not available. Waiting for it ....')
                    
        # Create a empty request
        self.req = SetBool.Request()

        # Init fag and req.data vars
        self.flag = False
        self.req.data = False
        self.failed_bt_nodes = []      

        # Subscription to /scan topic
        self.subscription = self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)
        self.subscription


    def bt_log_callback(self, msg):
        # Check if any of the distance values are less than 30cm
        for event in msg.event_log:
            node_name = event.node_name
            previous_status = event.previous_status
            current_status = event.current_status
            self.get_logger().info("Node: %s, Previous Status: %s, Current Status: %s" % (node_name, previous_status, current_status))

            # Check the FAILURE status to record
            if current_status == "FAILURE":
                self.get_logger().error("Node: %s, Previous Status: %s, Current Status: %s" % (node_name, previous_status, current_status))
                self.failed_bt_nodes.append(node_name)
                # Check if is not previously recording
                if not self.flag:
                    self.req.data = True
                    self.flag = True
                    self.send_request()

            #Stop recording if no node is in FAILURE status
            if len(self.failed_bt_nodes) == 0 and self.flag:
                self.req.data = False
                self.flag = False
                self.send_request()
                self.get_logger().info("All nodes have status SUCCESS, self.flag set to True")
       

    def send_request(self):
        # Send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)

    # Declare the node constructor
    client = BTStatusRecorderClientAsync()   

    while rclpy.ok():
        # Wait for a request to kill the node
        rclpy.spin(client)
        # Check if the service has sent a response
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Response state %r' % (response.success,))
            break

    # Delete client node
    client.destroy_node()

    # Shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
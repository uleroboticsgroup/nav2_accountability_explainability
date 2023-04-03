from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan

class ScanRecorderClientAsync(Node):
    DISTANCE_THRESHOLD = 1.35 # 1.35m

    def __init__(self):

        # Initialize the node
        super().__init__('scan_recorder_client')

        # Create the service client object
        self.client = self.create_client(SetBool, 'bag_recorder')

        # Check once per second the service's availability
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Service not available. Waiting for it ....')
                    
        # Create a empty request
        self.req = SetBool.Request()

        # QoS profiles subscription definition
        scan_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        # Init fag and req.data vars
        self.flag = False
        self.req.data = False
        

        # Subscription to /scan topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        self.subscription


    def scan_callback(self, msg):
        # Check if any of the distance values are less than 30cm
        ranges = msg.ranges
        min_range = min(ranges)
        
        if min_range < ScanRecorderClientAsync.DISTANCE_THRESHOLD and not self.flag:
            self.get_logger().info('Obstacle detected within {}cm'.format(min_range))
            self.req.data = True
            self.flag = True
            self.send_request()
            print(self.flag)
            print("{:.2f}".format(min_range))
        elif  min_range >= ScanRecorderClientAsync.DISTANCE_THRESHOLD and self.flag:           
            self.get_logger().info('Obstacle not detected')
            self.req.data = False
            self.flag = False
            self.send_request()
            print(self.flag)
            print("{:.2f}".format(min_range))
       

    def send_request(self):
        # Send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # Initialize the ROS communication
    rclpy.init(args=args)

    # Declare the node constructor
    client = ScanRecorderClientAsync()   

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



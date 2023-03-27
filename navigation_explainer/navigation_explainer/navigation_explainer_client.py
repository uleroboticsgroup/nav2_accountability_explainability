# Import the ExplanationServiceMessage from custom_interfaces package
from custom_interfaces.srv import ExplanationServiceMessage

# Import the ROS2 python client libraries
import rclpy
from rclpy.node import Node
import sys

class NavigationExplainerClientAsync(Node):

    def __init__(self):

        # Initialize the node
        super().__init__('navigation_explainer_client')

        # Create the service client object
        self.client = self.create_client(ExplanationServiceMessage, 'navigation_explainer')

        # Check once per second the service's availability
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Create a Empty request
        self.req = ExplanationServiceMessage.Request()
        

    def send_request(self):        
        # Send the request
        self.req.question = sys.argv[1]

        # Use sys.argv to get access to command line input arguments for the request.
        self.future = self.client.call_async(self.req)


def main(args=None):

    # Initialize the ROS communication
    rclpy.init(args=args)

    # Declare the node constructor
    client = NavigationExplainerClientAsync()

    # Run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # Checks the future for a response from the service while the system is running. 
                # If the service has sent a response, the result will be written to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info('%r' % (response.answer,))
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
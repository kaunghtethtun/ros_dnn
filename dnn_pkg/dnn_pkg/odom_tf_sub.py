import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage # Import TFMessage for tf topic

class OdometrySubscriber(Node):
    """
    A ROS2 node that subscribes to the 'odom' topic and prints odometry data.
    It also subscribes to the 'tf' topic to receive TF transformations.
    """
    def __init__(self):
        """
        Initializes the OdometrySubscriber node.
        """
        super().__init__('odometry_subscriber') # Initialize the ROS2 node with the name 'odometry_subscriber'

        # Odometry subscription
        self.odom_subscription = self.create_subscription(
            Odometry, # Message type to subscribe to
            'odom',   # Topic name to subscribe to
            self.odom_callback, # Callback function to handle incoming messages
            10        # QoS profile depth (queue size)
        )
        self.get_logger().info('Odometry subscriber node has been started and subscribing to "odom" topic.')

        # TF subscription
        self.tf_subscription = self.create_subscription(
            TFMessage, # Message type for TF transformations
            'tf',      # Topic name for TF transformations
            self.tf_callback, # Callback function to handle incoming TF messages
            10         # QoS profile depth
        )
        self.get_logger().info('Also subscribing to "tf" topic.')


    def odom_callback(self, msg: Odometry):
        """
        Callback function for the 'odom' topic.
        This function is called every time a new Odometry message is received.

        Args:
            msg (Odometry): The received Odometry message.
        """
        # Extract position data
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        position_z = msg.pose.pose.position.z

        # Extract orientation data (quaternion)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # yaw is typically the most relevant for 2D navigation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Extract linear velocity data
        linear_velocity_x = msg.twist.twist.linear.x
        linear_velocity_y = msg.twist.twist.linear.y
        linear_velocity_z = msg.twist.twist.linear.z

        # Extract angular velocity data
        angular_velocity_x = msg.twist.twist.angular.x
        angular_velocity_y = msg.twist.twist.angular.y
        angular_velocity_z = msg.twist.twist.angular.z

        # Log the received odometry data
        self.get_logger().info(f'\n--- Odometry Data ---'
                               f'\nPosition: x={position_x:.2f}, y={position_y:.2f}, z={position_z:.2f}'
                               f'\nOrientation (Euler): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}'
                               f'\nLinear Velocity: x={linear_velocity_x:.2f}, y={linear_velocity_y:.2f}, z={linear_velocity_z:.2f}'
                               f'\nAngular Velocity: x={angular_velocity_x:.2f}, y={angular_velocity_y:.2f}, z={angular_velocity_z:.2f}')

    def tf_callback(self, msg: TFMessage):
        """
        Callback function for the 'tf' topic.
        This function is called every time a new TFMessage is received.

        Args:
            msg (TFMessage): The received TFMessage.
        """
        if msg.transforms:
            # Log information about the first transform in the message
            transform = msg.transforms[0]
            self.get_logger().info(f'\n--- TF Data Received ---'
                                   f'\nChild Frame ID: {transform.child_frame_id}'
                                   f'\nHeader Frame ID: {transform.header.frame_id}'
                                   f'\nTranslation: x={transform.transform.translation.x:.2f}, '
                                   f'y={transform.transform.translation.y:.2f}, '
                                   f'z={transform.transform.translation.z:.2f}')
        else:
            self.get_logger().info('Received empty TFMessage.')


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    """
    rclpy.init(args=args) # Initialize the ROS2 client library
    odometry_subscriber = OdometrySubscriber() # Create an instance of the OdometrySubscriber node
    rclpy.spin(odometry_subscriber) # Keep the node alive until it's explicitly shut down
    odometry_subscriber.destroy_node() # Destroy the node when it's no longer needed
    rclpy.shutdown() # Shut down the ROS2 client library

if __name__ == '__main__':
    main()

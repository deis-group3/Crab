import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crab import gps_subscriber
from crab.gps_subscriber import MinimalSubscriber

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    #Shutdown
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
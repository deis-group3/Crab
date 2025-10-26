import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from crab.crab import Crab

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robotPositions',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.is_detected = False

        self.test_subscription = self.create_subscription(
            String,
            'action',
            self.test_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.crabs = []

    def test_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        message = msg.data.strip('"').split(',')
        velocities = message[-1].split(';')

        self.debug_vel_x = int(velocities[0]) / 255.0
        self.debug_vel_y = int(velocities[1]) / 255.0

        print("Vel:",self.debug_vel_x, self.debug_vel_y)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)

        clean_data = msg.data.strip('[]"')
        rows = clean_data.split(';')

        our_robot_row_front = None
        our_robot_row_back = None

        self.crabs.clear()

        for row in rows:
            row_splited = row.strip().split()
            if (int(row_splited[3]) == 2):
                our_robot_row_front = row_splited
            elif (int(row_splited[3]) == 4):
                our_robot_row_back = row_splited
            elif (int(row_splited[3]) != -1):
                crab = Crab(float(row_splited[0]) / 1000.0, float(row_splited[1]) / 1000.0)
                self.crabs.append(crab)
                print(int(row_splited[3]),": pos:", crab.pos)
                
        if (our_robot_row_front != None and our_robot_row_back != None):
            self.robot_pos = (
                (float(our_robot_row_front[0]) + float(our_robot_row_back[0])) / 2.0,
                (float(our_robot_row_front[1]) + float(our_robot_row_back[1])) / 2.0
            )

            self.robot_pos = (
                self.robot_pos[0] / 1000.0,
                self.robot_pos[1] / 1000.0
            )

            dx =  float(our_robot_row_front[0]) - float(our_robot_row_back[0])
            dy =  float(our_robot_row_front[1]) - float(our_robot_row_back[1])

            heading_radians = math.atan2(dy, dx)
            heading_degrees = math.degrees(heading_radians) % 360

            self.robot_heading = heading_radians
        
            print("x:", self.robot_pos[0], ", y:",  self.robot_pos[1], ", angle:", heading_degrees)

            self.is_detected = True
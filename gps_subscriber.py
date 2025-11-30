import math, time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from crab.crab import Crab

PIXEL_TO_METER_SCALE = 1.5

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
        self.new_position = False

        self.test_subscription = self.create_subscription(
            String,
            'action',
            self.test_listener_callback,
            10)
        self.test_subscription  # prevent unused variable warning
        self.debug_vel_x = 0
        self.debug_vel_y = 0

        self.crabs = []
        self.robot_pos = ()
        self.robot_pos_time = 0

        self.crab7_x = 0
        self.crab7_y = 0

    def to_physical_pos(self, pos):
        center = (1.0, 0.75)

        kx = -0.62
        ky = -0.42

        y_offset = 0.28
        x_offset = 0.55

        dx = center[0] - pos[0]
        dy = center[1] - pos[1]

        return (pos[0] + kx * dx + x_offset, pos[1] + ky * dy + y_offset)

    def test_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        message = msg.data.strip('"').split(',')
        velocities = message[-1].split(';')

        self.debug_vel_x = int(velocities[0]) / 255.0
        self.debug_vel_y = int(velocities[1]) / 255.0

        print("Vel:",self.debug_vel_x, self.debug_vel_y)

    def gps_hand_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        message = msg.data.split(',')
        self.hand_gesture = message[0]
        self.hand_pos = (float(message[1]), float(message[2]))
        #velocities = message[-1].split(';')

        #self.debug_vel_x = int(velocities[0]) / 255.0
        #self.debug_vel_y = int(velocities[1]) / 255.0

        #print("Vel:",self.debug_vel_x, self.debug_vel_y)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        #print(time.time())


        clean_data = msg.data.strip('[]"')
        rows = clean_data.split(';')

        our_robot_row_front = None
        our_robot_row_back = None

        self.crabs.clear()

        for row in rows:
            row_splited = row.strip().split()
            index = int(row_splited[3])
            if (index == 4):
                our_robot_row_front = row_splited
            elif (index == 5):
                our_robot_row_back = row_splited
            elif (index != -1):
                crab_pos = (float(row_splited[0]) / 1000.0 * PIXEL_TO_METER_SCALE, 
                            float(row_splited[1]) / 1000.0 * PIXEL_TO_METER_SCALE
                )

                #real_pos = self.to_physical_pos(crab_pos)
                crab = Crab(index, crab_pos[0], crab_pos[1])
                self.crabs.append(crab)
                #print(index,": pos:", crab.pos)

                if (index == 0):
                    self.crab7_x = crab.pos[0]
                    self.crab7_y = crab.pos[1]


        if (our_robot_row_front != None and our_robot_row_back != None):
            self.robot_pos = (
                (float(our_robot_row_front[0]) + float(our_robot_row_back[0])) / 2.0 / 1000.0 * PIXEL_TO_METER_SCALE,
                (float(our_robot_row_front[1]) + float(our_robot_row_back[1])) / 2.0 / 1000.0 * PIXEL_TO_METER_SCALE
            )

            #self.robot_pos = self.to_physical_pos(self.robot_pos)

            self.new_position = True
            self.robot_pos_time = time.time() - 0.65
            print("Robot: x=", self.robot_pos[0], ", y=",  self.robot_pos[1])
            
            dx =  float(our_robot_row_front[0]) - float(our_robot_row_back[0])
            dy =  float(our_robot_row_front[1]) - float(our_robot_row_back[1])

            heading_radians = math.atan2(dy, dx)
            heading_radians = (heading_radians + math.pi) % (2 * math.pi) - math.pi
            heading_degrees = math.degrees(heading_radians) % 360

            self.robot_heading = heading_radians
        
            print("Robot: angle=", heading_degrees)

            self.is_detected = True
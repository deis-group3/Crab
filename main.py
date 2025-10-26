import threading
import time, math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crab import gps_subscriber
from crab.gps_subscriber import MinimalSubscriber
from crab.motor_controller import MotorController

running = True
gps_subscriber = None
toolgate2 = False

state_forward = 0
state_turn = 1
state_backoff = 2
current_state = 0

def goTo(target_pos, target_heading):
    if (gps_subscriber.is_detected == False):
        return
    
    kp_ang = 2.0
    kp_pos = 0.2

    dx = target_pos[0] - gps_subscriber.robot_pos[0]
    dy = target_pos[1] - gps_subscriber.robot_pos[1]
    distance = math.sqrt(dx**2 + dy**2)
    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - gps_subscriber.robot_heading
    heading_error = target_heading - gps_subscriber.robot_heading
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    v_out = 0.0
    omega_out = 0.0

    if (abs(angle_error) > 0.1):
        omega_out = kp_ang * angle_error
    elif (distance > 0.02):
        v_out = min(2.0, kp_pos * distance)
        omega_out = kp_pos * angle_error
    elif (abs(heading_error) > 0.025):
        omega_out = kp_ang * heading_error
    else:
        v_out = 0.0
        omega_out = 0.0

    omega_out = max(min(omega_out, 0.15), -0.15)
    v_out = max(min(v_out, 0.5), -0.5)

    return (v_out, omega_out)


def check_for_exit(): 
    global running

    input("Press enter to stop the robot...\n")
    running = False

def run_gps_listener():
    rclpy.spin(gps_subscriber)

def main(args=None):
    global gps_subscriber, current_state

    exit_thread = threading.Thread(target=check_for_exit)
    exit_thread.daemon = True
    exit_thread.start()

    rclpy.init(args=args)
    gps_subscriber = MinimalSubscriber()

    gps_listen_thread = threading.Thread(target=run_gps_listener)
    gps_listen_thread.daemon = True
    gps_listen_thread.start()

    motor_controller = MotorController() 

    while (running):
        if (toolgate2):
            gps_subscriber.debug_vel_x = 0
            gps_subscriber.debug_vel_y = 0

            time.sleep(1.0)

            motor_controller.drive([gps_subscriber.debug_vel_x * 0.5, gps_subscriber.debug_vel_y * 0.5])
            continue

        
        if (gps_subscriber.is_detected):
            
            if (current_state == state_forward):
                closest_crab = None
                closest_dst = 1000000
                for crab in gps_subscriber.crabs:
                    dx = crab.pos[0] - gps_subscriber.robot_pos[0]
                    dy = crab.pos[1] - gps_subscriber.robot_pos[1]
                    dst = math.sqrt(dx**2 + dy**2)
                    if (dst < closest_dst):
                        closest_crab = crab
                        closest_dst = dst
                
                if (closest_crab != None and closest_dst < 0.3):
                    current_state = state_turn
                else:
                    motor_controller.drive((1.0, 1.0))
                    time.sleep(0.015)
                    motor_controller.drive((0.0, 0.0))
                    time.sleep(0.015)

            if (current_state == state_turn):
                motor_controller.drive((0.4, -0.4))
                time.sleep(2.2)
                current_state = state_backoff

            if (current_state == state_backoff):
                for i in range(100):
                    motor_controller.drive((1.0, 1.0))
                    time.sleep(0.015)
                    motor_controller.drive((0.0, 0.0))
                    time.sleep(0.015)
                current_state = state_forward
            

            ''' Stop at if to close

            to_close_to_crab = False
            for crab in gps_subscriber.crabs:
                dx = crab.pos[0] - gps_subscriber.robot_pos[0]
                dy = crab.pos[1] - gps_subscriber.robot_pos[1]
                dst = math.sqrt(dx**2 + dy**2)
                #print("dst:",dst)
                if (dst < 0.3):
                    to_close_to_crab = True

            if (to_close_to_crab == False):
                motor_controller.drive((1.0, 1.0))
                time.sleep(0.015)
                motor_controller.drive((0.0, 0.0))
                time.sleep(0.015)
            else:
                motor_controller.drive((0,0))
            '''     
        time.sleep(0.001)

    #Shutdown
    print("Shuting down...")
    motor_controller.drive((0, 0))
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
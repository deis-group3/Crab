import threading
import time, math
import pigpio
import numpy as np
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from crab import gps_subscriber
from crab.gps_subscriber import MinimalSubscriber
from crab.motor_controller import MotorController
from crab import encoders
from crab import odometry
from crab import run_planner_live

running = True
gps_subscriber = None
toolgate2 = False

state_forward = 0
state_turn = 1
state_backoff = 2
current_state = 0

client = None
hand_gesture = None
hand_pos = ()

WHEEL_RADIUS = 0.0325
WHEEL_BASE   = 0.15 - 0.04
TICKS_PER_REV = 190

OCC_PATH = "src/crab/crab/arena_occupancy.npy"
OCC_GRID_SCALE = 10

def to_grid_pos(x, y):
    grid_x = round(x * OCC_GRID_SCALE)
    grid_y = round(y * OCC_GRID_SCALE)

    return (grid_x, grid_y)

def to_real_pos(grid_x, grid_y):
    x = grid_x / OCC_GRID_SCALE
    y = grid_y / OCC_GRID_SCALE

    return (x, y)

def compute_motor_speeds(x, y, theta, tx, ty):
    dx = tx - x
    dy = ty - y
    target_angle = math.atan2(dy, dx)

    # Normalize angle error to [-pi, pi]
    angle_error = target_angle - theta
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    dist_error = math.hypot(dx, dy)

    # ------------------------------
    # Tunable parameters
    # ------------------------------
    ANGLE_THRESHOLD = math.radians(5)
    STOP_DISTANCE = 0.03 
    MAX_LINEAR = 1.0                     # softer top speed
    MAX_ANGULAR = 1.0                    # max turning speed

    # Smooth gains
    K_linear = 6.0                       # lower = smoother acceleration
    K_angular = 6.0                      # higher = snappier rotation

    if dist_error < STOP_DISTANCE:
        return 0.0, 0.0

    # ------------------------------
    # PRIORITIZE TURNING FIRST
    # If angle error is large, do rotation-in-place
    # ------------------------------
    if abs(angle_error) > ANGLE_THRESHOLD:
        v = 0.0
        w = K_angular * angle_error
    else:
        # Face is mostly aligned → allow forward movement
        v = K_linear * dist_error
        w = K_angular * angle_error   # damp turning at higher speeds

    # ------------------------------
    # Clamp speeds
    # ------------------------------
    v = max(-MAX_LINEAR, min(MAX_LINEAR, v))
    w = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))

    # ------------------------------
    # Differential drive conversion
    # ------------------------------
    left = v - w
    right = v + w

    # Normalize into [-1, 1]
    max_mag = max(1, abs(left), abs(right))
    left /= max_mag
    right /= max_mag

    return left, right

def check_for_exit(): 
    global running

    input("Press enter to stop the robot...\n")
    running = False

def run_gps_listener():
    rclpy.spin(gps_subscriber)

def gps_hand_listener():
    global client, hand_gesture, hand_pos
    
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(("127.0.0.1", 5000))

    while True:
        if (not running):
            break

        data = client.recv(1024)
        if not data:
            break

        #print("Received:", data.decode())

        message = data.decode().split(',')
        hand_gesture = message[0]
        hand_pos = (float(message[1]) / 1000 * 1.5, float(message[2]) / 1000 * 1.5)

        print("Hand pos:", hand_pos)

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

    gps_hand_listen_thead = threading.Thread(target=gps_hand_listener)
    gps_hand_listen_thead.deamon = True
    gps_hand_listen_thead.start()

    motor_controller = MotorController() 

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiod not running")

    enc_left = encoders.SingleEncoder(pi, encoders.ENC_LEFT, "Left")
    enc_right = encoders.SingleEncoder(pi, encoders.ENC_RIGHT, "Right")

    odom = odometry.DifferentialDriveOdometry(WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REV)
    
    occupancy = np.load(OCC_PATH)

    while (running):
        if (toolgate2):
            motor_controller.drive([gps_subscriber.debug_vel_x, gps_subscriber.debug_vel_y])
            continue

        if (not gps_subscriber.is_detected):
            time.sleep(0.01)
            continue

        if (gps_subscriber.new_position):
            gps_subscriber.new_position = False

            odom_at_gps_time = odom.get_pose_at_time(gps_subscriber.robot_pos_time)

            if (odom_at_gps_time is None):
                odom.setPos(gps_subscriber.robot_pos[0], gps_subscriber.robot_pos[1], gps_subscriber.robot_heading)
            else:

                def normalize_angle(a):
                    return (a + math.pi) % (2*math.pi) - math.pi

                err_x = gps_subscriber.robot_pos[0] - odom_at_gps_time[1]
                err_y = gps_subscriber.robot_pos[1] - odom_at_gps_time[2]
                err_theta = normalize_angle(gps_subscriber.robot_heading - odom_at_gps_time[3]) 

                estimate = (odom.x + err_x, odom.y + err_y, normalize_angle(odom.theta + err_theta))
                odom.setPos(estimate[0], estimate[1], estimate[2])

        #gesture, loc_m, loc_px = gesture_detector.read_gesture()

        #goal_pos = (gps_subscriber.crab7_x, gps_subscriber.crab7_y)
        if hand_gesture is not None:
            goal_pos = hand_pos
        else:
            goal_pos = (odom.x, odom.y)
            
        odom.update(enc_left.get_ticks(), enc_right.get_ticks())

        path = run_planner_live.astar(occupancy, to_grid_pos(odom.x, odom.y), to_grid_pos(goal_pos[0], goal_pos[1]))
        print("Robot grid:", to_grid_pos(odom.x, odom.y), "end grid:",to_grid_pos(goal_pos[0], goal_pos[1]))

        left_speed = 0
        right_speed = 0
        
        if not path:
            print("No path found...")
        else:
            target_index = 2
            if (len(path) > target_index):
                target_pos = to_real_pos(path[target_index][0], path[target_index][1])
                target_pos = (target_pos[0] + to_real_pos(0.5, 0.5)[0],
                              target_pos[1] + to_real_pos(0.5, 0.5)[1])
                #print("Target:", target_pos, path[target_index])
                
                left_speed, right_speed = compute_motor_speeds(odom.x, odom.y, odom.theta, target_pos[0], target_pos[1])
            else:
                #print("Moving to exact position:", goal_pos)
                left_speed, right_speed = compute_motor_speeds(odom.x, odom.y, odom.theta, goal_pos[0], goal_pos[1])
        
        motor_controller.drive((left_speed, right_speed))
        enc_left.set_direction(left_speed)
        enc_right.set_direction(right_speed)

        time.sleep(0.025)
        #motor_controller.drive((0, 0))
        #time.sleep(0.0008)

    #Shutdown
    print("Shuting down...")
    motor_controller.drive((0, 0))
    client.close()
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
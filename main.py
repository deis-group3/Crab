import threading
import time, math
import pigpio
import numpy as np

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

WHEEL_RADIUS = 0.0325
WHEEL_BASE   = 0.15 - 0.04 
TICKS_PER_REV = 190

OCC_PATH = "src/crab/crab/arena_occupancy.npy"

def to_grid_pos(x, y):
    grid_x = round(x * 10)
    grid_y = round(y * 10)

    return (grid_x, grid_y)

def to_real_pos(grid_x, grid_y):
    x = grid_x / 10
    y = grid_y / 10

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
    ANGLE_THRESHOLD = math.radians(4)
    STOP_DISTANCE = 0.05 
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
        w = K_angular * angle_error * 0.5   # damp turning at higher speeds

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

        odom.update(enc_left.get_ticks(), enc_right.get_ticks())

        if (gps_subscriber.new_position):
            gps_subscriber.new_position = False

            odom.setPos(gps_subscriber.robot_pos[0], gps_subscriber.robot_pos[1], gps_subscriber.robot_heading)

        path = run_planner_live.astar(occupancy, to_grid_pos(odom.x, odom.y), to_grid_pos(gps_subscriber.crab7_x, gps_subscriber.crab7_y))

        print("Robot grid:", to_grid_pos(odom.x, odom.y), " target grid:",to_grid_pos(gps_subscriber.crab7_x, gps_subscriber.crab7_y))

        left = 0
        right = 0

        if not path:
            print("No path found...")
        else:
            if (len(path) > 1):
                target_pos = to_real_pos(path[1][1], path[1][0])
                print("Target:",path[1])
                left, right = compute_motor_speeds(odom.x, odom.y, odom.theta, target_pos[0], target_pos[1])

        motor_controller.drive((left, right))
        enc_left.set_direction(left)
        enc_right.set_direction(right)

        time.sleep(0.012)
        motor_controller.drive((0, 0))
        time.sleep(0.002)

    #Shutdown
    print("Shuting down...")
    motor_controller.drive((0, 0))
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
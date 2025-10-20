#!/usr/bin/env python3
import sys
import time
import pigpio  # Python bindings for the pigpio library

def main():
    pi = pigpio.pi()  # connect to local pigpio daemon
    if not pi.connected:
        print("pigpio init failed", file=sys.stderr)
        return 1

    try:
        SERVO_GPIO = 18  # change if needed
        pi.set_mode(SERVO_GPIO, pigpio.OUTPUT) # set GPIO as output

        # Move to a few positions with brief pauses (µs pulse widths)
        positions = [1500, 1000, 2000, 1500]  # 500–2500 µs typical
        for pw in positions:
            print(f"Setting {pw} us")
            pi.set_servo_pulsewidth(SERVO_GPIO, pw)
            time.sleep(0.7)  # 700 ms for the servo to move

        # Stop sending pulses (releases torque)
        pi.set_servo_pulsewidth(SERVO_GPIO, 0)

        print("Done.")
        return 0

    finally:
        pi.stop()  # terminates connection to pigpio daemon

if __name__ == "__main__":
    sys.exit(main())

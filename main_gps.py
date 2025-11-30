from hand_gesture_ipcamera import HandGestureDetector

import threading
import socket

running = True

# Setup (once)
gesture_detector = HandGestureDetector(
    camera_url="http://192.168.1.2/mjpg/video.mjpg",
    debug=False,   # True if you want a window
    
)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("127.0.0.1", 5000))  # localhost only
server.listen(1)
conn, addr = server.accept()

def check_for_exit(): 
    global running

    input("Press enter to stop the robot...\n")
    running = False

exit_thread = threading.Thread(target=check_for_exit)
exit_thread.daemon = True
exit_thread.start()

# In your main control loop
while running:
    gesture, loc_m, loc_px = gesture_detector.read_gesture()

    if gesture is not None:
        print("Gesture:", gesture)
        print("Location pixels:", loc_px)

        msg = gesture + "," + str(loc_px[1]) + "," + str(loc_px[0])
        conn.send(msg.encode())

conn.close()
gesture_detector.release()
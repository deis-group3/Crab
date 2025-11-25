import cv2
import time
CAMERA_URL = "http://192.168.1.2/mjpg/video.mjpg" 
# Capture and save an overhead reference image from the webcam

cap = cv2.VideoCapture(CAMERA_URL) #Change to CAMERA_URL for network camera
ret, frame = cap.read() # Read a single frame
cap.release() # Release the capture

cv2.imwrite("arena_reference.jpg", frame) # Save the frame as an image file
print("Saved overhead reference image as 'arena_reference.jpg'.")


import cv2
import numpy as np

# === CONFIG ===
IMAGE_PATH = "arena_reference.jpg"  # your saved overhead image
GRID_ROWS = 36                      # 3.6 m 
GRID_COLS = 24                      # 2.4 m
OUTPUT_OCC_PATH = "arena_occupancy.npy" # occupancy grid output path
OUTPUT_START_PATH = "arena_start.npy" # start cell output path
OUTPUT_GOAL_PATH = "arena_goal.npy" # goal cell output path

# =============

img = cv2.imread(IMAGE_PATH) # load reference image
if img is None:
    raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

h, w, _ = img.shape # image dimensions
cell_h = h / GRID_ROWS # height of each grid cell
cell_w = w / GRID_COLS # width of each grid cell

# 0 = free, 1 = obstacle/border
occupancy = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8) # occupancy grid

start = None  # (row, col) # start cell
goal = None   # (row, col) # goal cell

current_mode = "obstacle"  # "obstacle", "start", "goal" # current editing mode


def draw_overlay(): 
    """Draw grid, obstacles, start, goal on top of the reference image."""
    vis = img.copy()

    # draw obstacle cells
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            if occupancy[r, c] == 1:
                x1, y1 = int(c * cell_w), int(r * cell_h)
                x2, y2 = int((c + 1) * cell_w), int((r + 1) * cell_h)
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), -1)

    # draw grid
    for r in range(1, GRID_ROWS):
        y = int(r * cell_h)
        cv2.line(vis, (0, y), (w, y), (0, 255, 0), 1)
    for c in range(1, GRID_COLS):
        x = int(c * cell_w)
        cv2.line(vis, (x, 0), (x, h), (0, 255, 0), 1)

    # draw start
    if start is not None:
        sx = int((start[1] + 0.5) * cell_w)
        sy = int((start[0] + 0.5) * cell_h)
        cv2.circle(vis, (sx, sy), 8, (0, 255, 0), -1)

    # draw goal
    if goal is not None:
        gx = int((goal[1] + 0.5) * cell_w)
        gy = int((goal[0] + 0.5) * cell_h)
        cv2.circle(vis, (gx, gy), 8, (255, 0, 0), -1)

    # show current mode text
    text = f"Mode: {current_mode.upper()}  |  1:Obstacle  2:Start  3:Goal  b:Border  c:Clear  w:Save  q:Quit"
    cv2.rectangle(vis, (0, 0), (w, 25), (0, 0, 0), -1)
    cv2.putText(vis, text, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return vis


def mouse_callback(event, x, y, flags, param):
    global occupancy, start, goal, current_mode
    if event == cv2.EVENT_LBUTTONDOWN:
        r = int(y / cell_h)
        c = int(x / cell_w)
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            if current_mode == "obstacle":
                # toggle obstacle
                occupancy[r, c] = 1 - occupancy[r, c]
                print(f"Toggled obstacle at cell (r={r}, c={c}) -> {occupancy[r, c]}")
            elif current_mode == "start":
                start = (r, c)
                print(f"Start set to cell: {start}")
            elif current_mode == "goal":
                goal = (r, c)
                print(f"Goal set to cell: {goal}")


def mark_borders_as_obstacles():
    global occupancy
    occupancy[0, :] = 1
    occupancy[-1, :] = 1
    occupancy[:, 0] = 1
    occupancy[:, -1] = 1
    print("Borders marked as obstacles.")


def clear_all():
    global occupancy, start, goal
    occupancy[:, :] = 0
    start = None
    goal = None
    print("Cleared occupancy, start, and goal.")


print("Controls:")
print("  Left click: act on cell depending on mode")
print("  Modes:")
print("    1 = obstacle mode (left click toggles obstacle)")
print("    2 = start mode    (left click sets START)")
print("    3 = goal mode     (left click sets GOAL)")
print("  b = mark borders as obstacles")
print("  c = clear map (occupancy, start, goal)")
print("  w = write/save .npy files and exit")
print("  q = quit WITHOUT saving")

cv2.namedWindow("label_arena", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("label_arena", mouse_callback)

while True:
    vis = draw_overlay()
    cv2.imshow("label_arena", vis)
    key = cv2.waitKey(20) & 0xFF

    if key == ord('1'):
        current_mode = "obstacle"
        print("Mode -> OBSTACLE")
    elif key == ord('2'):
        current_mode = "start"
        print("Mode -> START")
    elif key == ord('3'):
        current_mode = "goal"
        print("Mode -> GOAL")
    elif key == ord('b'):
        mark_borders_as_obstacles()
    elif key == ord('c'):
        clear_all()
    elif key == ord('w'):
        # save and exit
        np.save(OUTPUT_OCC_PATH, occupancy)
        print(f"Saved occupancy to {OUTPUT_OCC_PATH}")
        if start is not None:
            np.save(OUTPUT_START_PATH, np.array(start))
            print(f"Saved start to {OUTPUT_START_PATH}")
        else:
            print("WARNING: start not set, not saved.")

        if goal is not None:
            np.save(OUTPUT_GOAL_PATH, np.array(goal))
            print(f"Saved goal to {OUTPUT_GOAL_PATH}")
        else:
            print("WARNING: goal not set, not saved.")

        break
    elif key == ord('q'):
        print("Exiting without saving.")
        break

cv2.destroyAllWindows()

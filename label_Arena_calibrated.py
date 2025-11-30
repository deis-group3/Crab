import cv2
import numpy as np
import time
import os  # <--- Added to check if files exist

# ================== CONFIG ==================
CAMERA_URL = "http://192.168.1.2/mjpg/video.mjpg"

GRID_ROWS = 24
GRID_COLS = 36

IMAGE_PATH = "arena_reference.jpg"       # reference image (no warp)
OUTPUT_OCC_PATH = "arena_occupancy.npy"  # occupancy grid output path
OUTPUT_START_PATH = "arena_start.npy"    # start cell output path
OUTPUT_GOAL_PATH = "arena_goal.npy"      # goal cell output path

# Set this to True if you want to load previous files instead of creating new ones
LOAD_EXISTING = True  
# ===========================================


def capture_frame_from_camera():
    """Capture a single frame from the camera."""
    cap = cv2.VideoCapture(CAMERA_URL)
    # Warm up camera
    time.sleep(1)
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise RuntimeError("Could not read frame from camera")

    cv2.imwrite("arena_reference_raw.jpg", frame)
    print("Saved raw frame as 'arena_reference_raw.jpg'.")
    return frame


# ---------- STEP 1: Image Setup (Load or Capture) ----------
img = None

# Check if we should load the existing image
if LOAD_EXISTING and os.path.exists(IMAGE_PATH):
    print(f"Loading existing reference image: {IMAGE_PATH}")
    img = cv2.imread(IMAGE_PATH)
    if img is None:
        print("Error loading image. Falling back to camera capture.")

# If loading failed or was disabled, capture from camera
if img is None:
    print("Capturing new frame from camera...")
    raw_frame = capture_frame_from_camera()

    # Warp disabled: use raw captured frame as the reference image.
    img = raw_frame.copy()
    cv2.imwrite(IMAGE_PATH, img)
    print("Using raw frame (warp disabled).")


# ---------- STEP 2: Grid & Labeling (Load or New) ----------

h = int(1536/2)
w = int(2048/2)
img = cv2.resize(img, (w,h), dst=None, fx=None, fy=None, interpolation=cv2.INTER_LINEAR)
cell_h = h / float(GRID_ROWS)
cell_w = w / float(GRID_COLS)

# Initialize variables
occupancy = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)
start = None
goal = None

# LOAD PREVIOUS DATA if requested
if LOAD_EXISTING:
    # 1. Load Occupancy
    if os.path.exists(OUTPUT_OCC_PATH):
        try:
            loaded_occ = np.load(OUTPUT_OCC_PATH)
            if loaded_occ.shape == (GRID_ROWS, GRID_COLS):
                occupancy = loaded_occ
                print(f"Loaded existing occupancy grid from {OUTPUT_OCC_PATH}")
            else:
                print(f"Warning: Loaded occupancy shape {loaded_occ.shape} "
                      f"does not match config {(GRID_ROWS, GRID_COLS)}. Ignored.")
        except Exception as e:
            print(f"Failed to load occupancy: {e}")

    # # 2. Load Start
    # if os.path.exists(OUTPUT_START_PATH):
    #     try:
    #         start = tuple(np.load(OUTPUT_START_PATH)) # convert back to tuple
    #         print(f"Loaded start position: {start}")
    #     except:
    #         print("Could not load start position.")

    # # 3. Load Goal
    # if os.path.exists(OUTPUT_GOAL_PATH):
    #     try:
    #         goal = tuple(np.load(OUTPUT_GOAL_PATH)) # convert back to tuple
    #         print(f"Loaded goal position: {goal}")
    #     except:
    #         print("Could not load goal position.")


current_mode = "obstacle"

# ... (Rest of UI/Drawing code remains exactly the same) ...

def draw_overlay():
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

    # draw start/goal
    if start:
        sx, sy = int((start[1]+0.5)*cell_w), int((start[0]+0.5)*cell_h)
        cv2.circle(vis, (sx, sy), 8, (0, 255, 0), -1)
    if goal:
        gx, gy = int((goal[1]+0.5)*cell_w), int((goal[0]+0.5)*cell_h)
        cv2.circle(vis, (gx, gy), 8, (255, 0, 0), -1)

    text = f"Mode: {current_mode.upper()} | w:Save q:Quit"
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
                occupancy[r, c] = 1 - occupancy[r, c]
            elif current_mode == "start":
                start = (r, c)
            elif current_mode == "goal":
                goal = (r, c)

print("UI Loaded. Press 'w' to save changes, 'q' to quit.")

while True:
    cv2.imshow("label_arena", draw_overlay())
    cv2.setMouseCallback("label_arena", mouse_callback)
    cv2.resizeWindow("label_arena", w, h)
    key = cv2.waitKey(20) & 0xFF

    if key == ord('1'): current_mode = "obstacle"
    elif key == ord('2'): current_mode = "start"
    elif key == ord('3'): current_mode = "goal"
    elif key == ord('c'): occupancy[:,:] = 0; start=None; goal=None
    elif key == ord('w'):
        np.save(OUTPUT_OCC_PATH, occupancy)
        if start: np.save(OUTPUT_START_PATH, np.array(start))
        if goal: np.save(OUTPUT_GOAL_PATH, np.array(goal))
        print("Saved.")
        break
    elif key == ord('q'):
        break

cv2.destroyAllWindows()
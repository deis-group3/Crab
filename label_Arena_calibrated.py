import cv2
import numpy as np
import time

# ================== CONFIG ==================
CAMERA_URL = "http://192.168.1.2/mjpg/video.mjpg"

GRID_ROWS = 36   # 3.6 m
GRID_COLS = 24   # 2.4 m

IMAGE_PATH = "arena_reference.jpg"       # warped overhead image
OUTPUT_OCC_PATH = "arena_occupancy.npy"  # occupancy grid output path
OUTPUT_START_PATH = "arena_start.npy"    # start cell output path
OUTPUT_GOAL_PATH = "arena_goal.npy"      # goal cell output path
# ===========================================


def capture_frame_from_camera():
    """Capture a single frame from the camera."""
    cap = cv2.VideoCapture(CAMERA_URL)
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise RuntimeError("Could not read frame from camera")

    cv2.imwrite("arena_reference_raw.jpg", frame)
    print("Saved raw frame as 'arena_reference_raw.jpg'.")
    return frame


def select_corners_on_image(frame):
    """
    Let the user click 4 corners on the image in this order:
    top-left, top-right, bottom-right, bottom-left.

    Returns np.float32 array of shape (4, 2) with the points,
    or None if user cancels.
    """
    clone = frame.copy()
    display = clone.copy()
    points = []

    window_name = "select_corners"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    instructions = (
        "Click corners in order: TL, TR, BR, BL. "
        "'r' = reset, 'q' = cancel."
    )

    def mouse_callback(event, x, y, flags, param):
        nonlocal points, display
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points) < 4:
                points.append((x, y))
                # draw point and index
                cv2.circle(display, (x, y), 6, (0, 0, 255), -1)
                cv2.putText(
                    display,
                    str(len(points)),
                    (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                print(f"Corner {len(points)}: ({x}, {y})")

    cv2.setMouseCallback(window_name, mouse_callback)

    print(instructions)
    while True:
        disp_with_text = display.copy()
        cv2.rectangle(disp_with_text, (0, 0), (1000, 30), (0, 0, 0), -1)
        cv2.putText(
            disp_with_text,
            instructions,
            (5, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            1,
        )

        cv2.imshow(window_name, disp_with_text)
        key = cv2.waitKey(20) & 0xFF

        if key == ord('q'):
            print("Corner selection canceled, using raw image.")
            cv2.destroyWindow(window_name)
            return None

        if key == ord('r'):
            print("Resetting corner selection.")
            points = []
            display = clone.copy()

        if len(points) == 4:
            print("4 corners selected.")
            cv2.destroyWindow(window_name)
            return np.float32(points)


def warp_arena(frame, src_pts):
    """
    Warp the arena given 4 source points into a rectangle
    with aspect ratio GRID_ROWS x GRID_COLS.
    """
    # choose pixels per cell
    cell_px = 20  # you can change this if you want larger/smaller image
    W = GRID_COLS * cell_px   # width in pixels
    H = GRID_ROWS * cell_px   # height in pixels

    # destination points: perfect rectangle
    dst_pts = np.float32([
        [0,     0],
        [W - 1, 0],
        [W - 1, H - 1],
        [0,     H - 1],
    ])

    # perspective transform
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(frame, M, (W, H))

    cv2.imwrite(IMAGE_PATH, warped)
    print(f"Saved warped arena image as '{IMAGE_PATH}'. "
          f"Size: {W}x{H} pixels")
    return warped


# ---------- STEP 1: capture and warp arena ----------
raw_frame = capture_frame_from_camera()
src_pts = select_corners_on_image(raw_frame)

if src_pts is not None and len(src_pts) == 4:
    img = warp_arena(raw_frame, src_pts)
else:
    # fall back to raw frame (no warp)
    img = raw_frame.copy()
    cv2.imwrite(IMAGE_PATH, img)
    print(f"Saved unwarped image as '{IMAGE_PATH}'.")

# ---------- STEP 2: grid + labeling UI (original logic) ----------

# load reference image (warped or raw)
img = cv2.imread(IMAGE_PATH)
if img is None:
    raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")

h, w, _ = img.shape
cell_h = h / GRID_ROWS
cell_w = w / GRID_COLS

# 0 = free, 1 = obstacle/border
occupancy = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.uint8)

start = None  # (row, col)
goal = None   # (row, col)

current_mode = "obstacle"  # "obstacle", "start", "goal"


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
    text = (
        f"Mode: {current_mode.upper()}  |  "
        "1:Obstacle  2:Start  3:Goal  b:Border  c:Clear  w:Save  q:Quit"
    )
    cv2.rectangle(vis, (0, 0), (w, 25), (0, 0, 0), -1)
    cv2.putText(
        vis,
        text,
        (5, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
    )

    return vis


def mouse_callback(event, x, y, flags, param):
    global occupancy, start, goal, current_mode
    if event == cv2.EVENT_LBUTTONDOWN:
        r = int(y / cell_h)
        c = int(x / cell_w)
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            if current_mode == "obstacle":
                occupancy[r, c] = 1 - occupancy[r, c]
                print(f"Toggled obstacle at cell (r={r}, c={c}) "
                      f"-> {occupancy[r, c]}")
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

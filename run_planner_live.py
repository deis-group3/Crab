import cv2
import numpy as np
import heapq

# ========= CONFIG =========
OCC_PATH   = "arena_occupancy.npy"
START_PATH = "arena_start.npy"
GOAL_PATH  = "arena_goal.npy"
CAMERA_URL = "http://192.168.1.2/mjpg/video.mjpg" #IP camera URL (if needed)

CAM_INDEX = 0       # usually 0 for default camera
WINDOW_NAME = "Arena A* Live" # window name
# ==========================


# ---------- A* on grid ----------
def astar(occupancy, start, goal):
    """
    occupancy: 2D numpy array (rows x cols), 0 = free, 1 = obstacle
    start, goal: (row, col)
    returns: list of (row, col) cells, including start and goal; [] if no path
    """
    rows, cols = occupancy.shape

    def in_bounds(r, c):
        return 0 <= r < rows and 0 <= c < cols

    def is_free(r, c):
        return occupancy[r, c] == 0

    def heuristic(a, b):
        (r1, c1), (r2, c2) = a, b
        # Manhattan distance
        return abs(r1 - r2) + abs(c1 - c2)

    # 4-connected neighbors
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start))

    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_heap:
        f, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue

        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        closed.add(current)
        cr, cc = current

        for dr, dc in neighbors:
            nr, nc = cr + dr, cc + dc
            neighbor = (nr, nc)

            if not in_bounds(nr, nc):
                continue
            if not is_free(nr, nc):
                continue
            if neighbor in closed:
                continue

            tentative_g = g + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_heap, (f_score, tentative_g, neighbor))

    # no path
    return []


def main():
    # ---- Load map, start, goal ----
    occupancy = np.load(OCC_PATH)
    start = tuple(np.load(START_PATH))
    goal = tuple(np.load(GOAL_PATH))

    rows, cols = occupancy.shape
    print("Grid size:", rows, "x", cols)
    print("Start cell:", start)
    print("Goal cell:", goal)

    # ---- Run A* once (map is static) ----
    path = astar(occupancy, start, goal)
    if not path:
        print("WARNING: no path found from start to goal!")
    else:
        print("Path length:", len(path), "cells")

    # ---- Open camera ----
    cap = cv2.VideoCapture(CAMERA_URL)
    if not cap.isOpened():
        print("Could not open camera")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Optional: resize for speed / consistency
        # You can match the resolution you used when labeling, but not required.
        frame = cv2.resize(frame, (640, 480))

        h, w, _ = frame.shape
        cell_h = h / rows
        cell_w = w / cols

        vis = frame.copy()

        # ---- Draw obstacles (red cells) ----
        for r in range(rows):
            for c in range(cols):
                if occupancy[r, c] == 1:
                    x1 = int(c * cell_w)
                    y1 = int(r * cell_h)
                    x2 = int((c + 1) * cell_w)
                    y2 = int((r + 1) * cell_h)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), -1)

        # ---- Draw grid (green lines) ----
        for r in range(1, rows):
            y = int(r * cell_h)
            cv2.line(vis, (0, y), (w, y), (0, 255, 0), 1)
        for c in range(1, cols):
            x = int(c * cell_w)
            cv2.line(vis, (x, 0), (x, h), (0, 255, 0), 1)

        # ---- Draw path (blue polyline) ----
        if path:
            pts = []
            for (r, c) in path:
                cx = int((c + 0.5) * cell_w)
                cy = int((r + 0.5) * cell_h)
                pts.append([cx, cy])
            pts = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(vis, [pts], isClosed=False, color=(255, 0, 0), thickness=2)

        # ---- Draw start (green circle) and goal (cyan circle) ----
        sx = int((start[1] + 0.5) * cell_w)
        sy = int((start[0] + 0.5) * cell_h)
        gx = int((goal[1] + 0.5) * cell_w)
        gy = int((goal[0] + 0.5) * cell_h)

        cv2.circle(vis, (sx, sy), 8, (0, 255, 0), -1)    # start: green
        cv2.circle(vis, (gx, gy), 8, (255, 255, 0), -1)  # goal: yellow/cyan

        # ---- Show info text ----
        text = "q: quit"
        cv2.putText(vis, text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow(WINDOW_NAME, vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

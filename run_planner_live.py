import numpy as np
import heapq

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
        # Chebyshev distance (8-connected) maximum of absolute differences
        return max(abs(r1 - r2), abs(c1 - c2))

    # 8-connected neighbors (including diagonals)
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

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
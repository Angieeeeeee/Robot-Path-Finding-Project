from brushfirealgorithm import planPath
from robotpath import (
    maze, rows, cols, obstacles, start, end,
    forward, turnLeft, turnRight )

NORTH = (0, -1)
EAST  = (1, 0)
SOUTH = (0, 1)
WEST  = (-1, 0)
HEADINGS = [NORTH, EAST, SOUTH, WEST]

def rotateLeft(h):   return HEADINGS[(HEADINGS.index(h) - 1) % 4]
def rotateRight(h):  return HEADINGS[(HEADINGS.index(h) + 1) % 4]

def turnRobot(h_from, h_to):
    if h_to == h_from:
        return h_to
    if h_to == rotateRight(h_from):
        turnLeft(); return h_to
    if h_to == rotateLeft(h_from):
        turnRight();  return h_to
    # 180:
    turnRight(); turnRight()
    return h_to

def mazeGrid(maze, obstacles):
    H, W = len(maze), len(maze[0])
    g = [[0 for _ in range(W)] for __ in range(H)]
    for y in range(H):
        for x in range(W):
            if maze[y][x] == 1:
                g[y][x] = 1
    for (ox, oy) in obstacles:
        if 0 <= ox < W and 0 <= oy < H:
            g[oy][ox] = 1
    return g

def expandedSteps(path):
    """Expand waypoints into 4-connected unit steps."""
    if not path or len(path) == 1:
        return path
    out = [path[0]]
    for i in range(1, len(path)):
        x0, y0 = out[-1]
        x1, y1 = path[i]
        dx = x1 - x0
        dy = y1 - y0
        if dx != 0 and dy != 0:
            raise ValueError("Path contains diagonal move: %r -> %r" % ((x0,y0),(x1,y1)))
        step_x = 0 if dx == 0 else (1 if dx > 0 else -1)
        step_y = 0 if dy == 0 else (1 if dy > 0 else -1)
        steps = abs(dx) + abs(dy)
        for _ in range(steps):
            x0 += step_x
            y0 += step_y
            out.append((x0, y0))
    return out

def drivePath(path, startHeading=EAST):
    if not path or len(path) == 1:
        return
    # Ensure we have 1-tile steps
    steps = expandedSteps(path)

    heading = startHeading
    for i in range(1, len(steps)):
        x0, y0 = steps[i-1]
        x1, y1 = steps[i]
        step = (x1 - x0, y1 - y0)
        if step not in HEADINGS:
            raise ValueError("Non 4-connected step %r at %d: %r -> %r" %
                             (step, i, steps[i-1], steps[i]))
        heading = turnRobot(heading, step)
        forward()

grid = mazeGrid(maze, obstacles)
try:
    raw, smooth = planPath(grid, start, end, safetyMargin=0, return_raw=True)
except ValueError as e:
    print("Planning error:", e)

tilePath = raw if raw else []
if not tilePath:
    print("No path found.");

print("Planned raw path:", tilePath)
drivePath(tilePath, startHeading=EAST)

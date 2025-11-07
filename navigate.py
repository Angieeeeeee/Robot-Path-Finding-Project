from brushfirealgorithm import plan_path
from robotpath import (
    maze, rows, cols, obstacles, start, end,
    forward, turnLeft, turnRight
)

NORTH = (0, -1)
EAST  = (1, 0)
SOUTH = (0, 1)
WEST  = (-1, 0)
HEADINGS = [NORTH, EAST, SOUTH, WEST]

def heading_index(h): return HEADINGS.index(h)
def rotate_left(h):   return HEADINGS[(heading_index(h) - 1) % 4]
def rotate_right(h):  return HEADINGS[(heading_index(h) + 1) % 4]

def turn_robot_from_to(h_from, h_to):
    if h_to == h_from:
        return h_to
    if h_to == rotate_right(h_from):
        turnRight(); return h_to
    if h_to == rotate_left(h_from):
        turnLeft();  return h_to
    # 180:
    turnRight(); turnRight()
    return h_to

def grid_from_maze(maze, obstacles):
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

def expand_to_unit_steps(path):
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

def drive_path(path, start_heading=EAST):
    if not path or len(path) == 1:
        return
    # Ensure we have 1-tile steps
    steps = expand_to_unit_steps(path)

    heading = start_heading
    for i in range(1, len(steps)):
        x0, y0 = steps[i-1]
        x1, y1 = steps[i]
        step = (x1 - x0, y1 - y0)
        if step not in HEADINGS:
            raise ValueError("Non 4-connected step %r at %d: %r -> %r" %
                             (step, i, steps[i-1], steps[i]))
        heading = turn_robot_from_to(heading, step)
        forward()

def main():
    grid = grid_from_maze(maze, obstacles)
    try:
        raw, smooth = plan_path(grid, start, end, safety_margin=0, return_raw=True)
    except ValueError as e:
        print("Planning error:", e)
        return

    tile_path = raw if raw else []
    if not tile_path:
        print("No path found."); return

    print("Planned raw path:", tile_path)
    drive_path(tile_path, start_heading=EAST)

if __name__ == "__main__":
    main()

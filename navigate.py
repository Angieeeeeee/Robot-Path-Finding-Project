# navigate.py
# Tie brushfire path planning to your robot movement functions.

from brushfirealgorithm import plan_path
from robotpath import (
    maze, rows, cols, obstacles, start, end,
    forward, turnLeft, turnRight
)

# -----------------------------------------
# Helpers: heading & turning on a grid
# -----------------------------------------

# We'll use 4-connected moves; start facing EAST by default.
# Headings are unit vectors: (dx, dy)
NORTH = (0, -1)
EAST  = (1, 0)
SOUTH = (0, 1)
WEST  = (-1, 0)
HEADINGS = [NORTH, EAST, SOUTH, WEST]  # order matters for left/right math

def heading_index(h):
    return HEADINGS.index(h)

def rotate_left(h):
    # one step CCW in the list
    return HEADINGS[(heading_index(h) - 1) % 4]

def rotate_right(h):
    # one step CW in the list
    return HEADINGS[(heading_index(h) + 1) % 4]

def rotate_180(h):
    return HEADINGS[(heading_index(h) + 2) % 4]

def turn_robot_from_to(h_from, h_to):
    """Emit the minimal sequence of turns to go from h_from to h_to using
    your timed turn primitives.
    """
    if h_to == h_from:
        return h_to
    if h_to == rotate_right(h_from):
        turnRight()
        return h_to
    if h_to == rotate_left(h_from):
        turnLeft()
        return h_to
    # 180 turn
    turnRight()
    turnRight()
    return h_to

# -----------------------------------------
# Build grid from your maze + obstacles
# -----------------------------------------

def grid_from_maze(maze, obstacles):
    # Make a fresh 0/1 grid the planner expects
    H, W = len(maze), len(maze[0])
    g = [[0 for _ in range(W)] for __ in range(H)]
    # Copy any existing 1s from maze (if you’re using it that way)
    for y in range(H):
        for x in range(W):
            if maze[y][x] == 1:
                g[y][x] = 1
    # Add obstacles provided as (x, y) tuples
    for (ox, oy) in obstacles:
        if 0 <= ox < W and 0 <= oy < H:
            g[oy][ox] = 1
    return g

# -----------------------------------------
# Execute a tile path with your primitives
# -----------------------------------------

def drive_path(path, start_heading=EAST):
    """Given a list of (x,y) tiles, move the robot along the path.
       Assumes each step is a +/-1 move in x or y (4-connected)."""
    if not path or len(path) == 1:
        return

    heading = start_heading
    for i in range(1, len(path)):
        x0, y0 = path[i-1]
        x1, y1 = path[i]
        step = (x1 - x0, y1 - y0)  # desired heading for this segment

        # Map step vector onto one of the 4 headings
        if step not in HEADINGS:
            raise ValueError(f"Non 4-connected step {step} at segment {i}: {path[i-1]} -> {path[i]}")

        # Turn as needed, then go forward one tile
        heading = turn_robot_from_to(heading, step)
        forward()

# -----------------------------------------
# Main: plan + drive
# -----------------------------------------

def main():
    # 1) Build the occupancy grid (0=free, 1=obstacle)
    grid = grid_from_maze(maze, obstacles)

    # 2) Plan using Brushfire + Wavefront
    #    Adjust safety_margin if you want “inflated” obstacles.
    try:
        tile_path = plan_path(grid, start, end, safety_margin=0, return_raw=False)
    except ValueError as e:
        print("Planning error:", e)
        return

    if not tile_path:
        print("No path found.")
        return

    print("Planned path:", tile_path)

    # 3) Execute on robot (assumes you start facing EAST; change if needed)
    drive_path(tile_path, start_heading=EAST)

if __name__ == "__main__":
    main()
from collections import deque
from typing import List, Tuple

Grid = List[List[int]]
Point = Tuple[int, int]

# ---------------- Public API ----------------

def plan_path(
    grid: Grid,
    start: Point,
    goal: Point,
    safety_margin: int = 0,
    return_raw: bool = False,
):
    """
    Plan a safe grid path using Brushfire (clearance) + Wavefront (steps-to-goal).

    Args:
        grid: HxW matrix with 0=free, 1=obstacle.
        start: (x,y) tile coordinates.
        goal: (x,y) tile coordinates.
        safety_margin: inflate obstacles by N tiles (0 = none).
        return_raw: if True, returns (raw_path, smoothed_path); else just smoothed_path.

    Returns:
        path: list of (x,y) tiles, smoothed unless return_raw=True.
        Or (raw, smooth) if return_raw=True.

    Raises:
        ValueError: if start/goal out of bounds or lie in an inflated obstacle.
    """
    H, W = len(grid), len(grid[0])
    _assert_point(start, W, H, "start")
    _assert_point(goal,  W, H, "goal")

    g = inflate_obstacles(grid, safety_margin)

    # Re-check after inflation
    sx, sy = start; gx, gy = goal
    if g[sy][sx] == 1:
        raise ValueError("Start lies in inflated obstacle; reduce safety_margin or move start.")
    if g[gy][gx] == 1:
        raise ValueError("Goal lies in inflated obstacle; reduce safety_margin or move goal.")

    clearance = brushfire_obstacle_distance(g)
    wave = wavefront_from_goal(g, goal)
    raw = extract_path_with_clearance(start, goal, wave, clearance)
    if not raw:
        return [] if not return_raw else ([], [])
    smooth = smooth_path(raw)
    return (raw, smooth) if return_raw else smooth

# --------------- Internals (unchanged logic) ---------------

def _assert_point(p: Point, W: int, H: int, name: str):
    x, y = p
    if not (0 <= x < W and 0 <= y < H):
        raise ValueError(f"{name} {p} out of bounds for grid {W}x{H}")

def inb(x:int, y:int, W:int, H:int) -> bool:
    return 0 <= x < W and 0 <= y < H

def brushfire_obstacle_distance(grid: Grid) -> Grid:
    H, W = len(grid), len(grid[0])
    INF = 10**9
    dist = [[INF]*W for _ in range(H)]
    q = deque()

    for y in range(H):
        for x in range(W):
            if grid[y][x] == 1:
                dist[y][x] = 0
                q.append((x, y))

    NX, NY = (1, -1, 0, 0), (0, 0, 1, -1)
    while q:
        ux, uy = q.popleft()
        for k in range(4):
            vx, vy = ux + NX[k], uy + NY[k]
            if not inb(vx, vy, W, H) or grid[vy][vx] == 1:
                continue
            cand = dist[uy][ux] + 1
            if cand < dist[vy][vx]:
                dist[vy][vx] = cand
                q.append((vx, vy))
    return dist

def wavefront_from_goal(grid: Grid, goal: Point) -> Grid:
    gx, gy = goal
    H, W = len(grid), len(grid[0])
    INF = 10**9
    wave = [[(-1 if grid[y][x] == 1 else INF) for x in range(W)] for y in range(H)]
    q = deque([(gx, gy)])
    wave[gy][gx] = 0

    NX, NY = (1, -1, 0, 0), (0, 0, 1, -1)
    while q:
        ux, uy = q.popleft()
        for k in range(4):
            vx, vy = ux + NX[k], uy + NY[k]
            if not inb(vx, vy, W, H) or wave[vy][vx] == -1:
                continue
            cand = wave[uy][ux] + 1
            if cand < wave[vy][vx]:
                wave[vy][vx] = cand
                q.append((vx, vy))
    return wave

def extract_path_with_clearance(start: Point, goal: Point, wave: Grid, clearance: Grid) -> List[Point]:
    sx, sy = start
    gx, gy = goal
    H, W = len(wave), len(wave[0])
    INF = 10**9
    if wave[sy][sx] >= INF:
        return []

    path = [(sx, sy)]
    x, y = sx, sy
    NX, NY = (1, -1, 0, 0), (0, 0, 1, -1)

    # prefer continuing direction when clearance ties (reduces wiggle)
    def direction(prev, cur):
        if prev is None: return None
        (px, py), (cx, cy) = prev, cur
        return (cx - px, cy - py)

    prev = None
    while not (x == gx and y == gy):
        cur_wave = wave[y][x]
        best = None
        best_clear = -1
        best_continue = -1
        cur_dir = direction(prev, (x, y))

        # ideal neighbors: wave = cur-1 (shortest)
        for k in range(4):
            vx, vy = x + NX[k], y + NY[k]
            if not inb(vx, vy, W, H): 
                continue
            if wave[vy][vx] == cur_wave - 1:
                c = clearance[vy][vx]
                cont = 1 if cur_dir and (vx - x, vy - y) == cur_dir else 0
                if (c > best_clear) or (c == best_clear and cont > best_continue):
                    best = (vx, vy); best_clear = c; best_continue = cont

        # fallback: any lower wave
        if best is None:
            for k in range(4):
                vx, vy = x + NX[k], y + NY[k]
                if inb(vx, vy, W, H) and wave[vy][vx] < cur_wave:
                    c = clearance[vy][vx]
                    cont = 1 if cur_dir and (vx - x, vy - y) == cur_dir else 0
                    if (c > best_clear) or (c == best_clear and cont > best_continue):
                        best = (vx, vy); best_clear = c; best_continue = cont

        if best is None:
            return []

        prev = (x, y)
        x, y = best
        path.append((x, y))
        if len(path) > W * H:
            break
    return path

def smooth_path(path: List[Point]) -> List[Point]:
    if len(path) <= 2:
        return path[:]
    sm = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        if (x1 - x0, y1 - y0) != (x2 - x1, y2 - y1):
            sm.append(path[i])
    sm.append(path[-1])
    return sm

def inflate_obstacles(grid: Grid, margin: int = 1) -> Grid:
    if margin <= 0:
        return [row[:] for row in grid]
    H, W = len(grid), len(grid[0])
    out = [row[:] for row in grid]
    for y in range(H):
        for x in range(W):
            if grid[y][x] == 1:
                for dy in range(-margin, margin + 1):
                    for dx in range(-margin, margin + 1):
                        vx, vy = x + dx, y + dy
                        if inb(vx, vy, W, H):
                            out[vy][vx] = 1
    return out
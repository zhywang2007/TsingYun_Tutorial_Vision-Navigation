"""Local planner: Pure Pursuit controller."""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

import math
def local_plan(
    current_pose: Tuple[float, float],
    max_speed: float,
    max_accel: float,
    global_path: List[Tuple[float, float]],
    costmap: np.ndarray = None,
) -> Tuple[float, float]:
    """
    Convert the next chunk of the global path into a velocity command.

    Parameters
    ----------
    current_pose : Tuple[float, float], (x, y)
        Current robot position in world (grid-unit) coordinates.
    max_speed : float
        Maximum allowed speed magnitude (grid units / second). The returned
        command vector should not exceed this length.
    max_accel : float
        Maximum allowed acceleration. You may ignore this if the world's
        `step()` already enforces a ramp; otherwise use it to compute a
        feasible command from the current velocity.
    global_path : List[Tuple[float, float]], list of (x, y) waypoints from start to goal
        Waypoints from the global planner, ordered from current pose to goal.
        May be empty if no path was found — in that case return `(0.0, 0.0)`.

    Returns
    -------
    cmd_vx, cmd_vy : float, float
        Desired world-frame velocity in grid units per second. The world step
        will clip this to `max_speed` and ramp toward it at `max_accel`, so
        returning a "pointing at the look-ahead" vector scaled to `max_speed`
        is usually the right move.

    Notes
    -----
    - Pure Pursuit recipe:
        1. Find the look-ahead point on `global_path`: walk forward from the
           closest waypoint to `current_pose` until the cumulative distance
           exceeds a look-ahead radius `Ld` (a tuning constant, e.g. 1.5-2.5
           grid units). If you reach the last waypoint first, use it.
        2. The command direction is `(look_ahead - current_pose)`, normalized.
        3. The command speed is `max_speed` (or a slowed value if the
           remaining path length is short, to ease into the goal).
    - Optional: More complex local programming methods (such as Dynamic
      Window Approach) can be used, or more complex model prediction methods
      (such as MPPI) can be tried.
    """
    # TODO: Implement Pure Pursuit controller.
    if not global_path:
        return 0.0, 0.0
    Ld=2
    # 1. 找最近点索引
    best_idx = 0
    min_dist = float('inf')
    for i, wp in enumerate(global_path):
        d = math.hypot(current_pose[0]-wp[0], current_pose[1]-wp[1])
        if d < min_dist:
            min_dist = d
            best_idx = i

    # 2. 累积距离找前瞻点
    cum = 0.0
    idx = best_idx
    while idx < len(global_path)-1 and cum < Ld:
        p1 = global_path[idx]
        p2 = global_path[idx+1]
        cum += math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        idx += 1
    lookahead = global_path[idx]

    # 3. 方向向量
    dx = lookahead[0] - current_pose[0]
    dy = lookahead[1] - current_pose[1]
    length = math.hypot(dx, dy)
    if length < 1e-3:
        return 0.0, 0.0

    # 4. 速度大小（接近终点时减速）
    goal_dist = math.hypot(global_path[-1][0]-current_pose[0],
                           global_path[-1][1]-current_pose[1])
    if goal_dist < 2*Ld:
        speed = max_speed * (goal_dist / 4*Ld)
    else:
        speed = max_speed

    vx = dx / length * speed
    vy = dy / length * speed
    return vx, vy

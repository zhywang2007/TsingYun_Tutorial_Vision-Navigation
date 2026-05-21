"""Global path planner: A* search on a costmap grid."""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def global_plan(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    costmap: np.ndarray,
) -> List[Tuple[float, float]]:
    """
    Run path search over `costmap` to find a path from `start` to `goal`.

    Parameters
    ----------
    start : Tuple[float, float], (x, y)
        Start position in world (grid-unit) coordinates. `costmap[int(y), int(x)]`
        is the cell containing this point.
    goal : Tuple[float, float], (x, y)
        Goal position in the same coordinate system.
    costmap : np.ndarray, shape (rows, cols), dtype uint8
        Per-cell traversal cost. Cells with large cost are treated as impassable
        (lethal). Otherwise the cell's cost is added to the step cost so the
        planner is biased away from inflated/dangerous areas.

    Returns
    -------
    path : List[Tuple[float, float]], list of (x, y) waypoints from start to goal.
        World-coordinate waypoints from start to goal, inclusive of both ends.
        Returns [] if no path exists or if start/goal lie inside a lethal cell.

    Notes
    -----
    - Use 8-connectivity (N/S/E/W + 4 diagonals). Step cost between adjacent
      cells should be `dist + cell_cost`, where `dist` is 1.0 for cardinal moves
      and sqrt(2) for diagonals.
    - Use either a shortest path algorithm (like Dijkstra) or a heuristic search
      algorithm (like A*). If using A*, a good heuristic is the octile distance
      (diagonal distance) or Euclidean distance.
    """
    rows, cols = costmap.shape
    lethal_threshold = 250

    # World to grid (x -> col, y -> row)
    start_col = int(round(start[0]))
    start_row = int(round(start[1]))
    goal_col = int(round(goal[0]))
    goal_row = int(round(goal[1]))

    # Check bounds and obstacle status
    if not (0 <= start_row < rows and 0 <= start_col < cols):
        return []
    if not (0 <= goal_row < rows and 0 <= goal_col < cols):
        return []
    if costmap[start_row, start_col] >= lethal_threshold:
        return []
    if costmap[goal_row, goal_col] >= lethal_threshold:
        return []

    # 8 directions: (dr, dc, step_len)
    directions = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (-1, -1, np.sqrt(2))
    ]

    INF = 1e9
    dist = np.full((rows, cols), INF, dtype=float)
    parent = np.full((rows, cols, 2), -1, dtype=int)
    visited = np.zeros((rows, cols), dtype=bool)

    dist[start_row, start_col] = 0.0

    for _ in range(rows * cols):
        # Find unvisited node with smallest dist
        min_dist = INF
        u = (-1, -1)
        for i in range(rows):
            for j in range(cols):
                if not visited[i, j] and dist[i, j] < min_dist:
                    min_dist = dist[i, j]
                    u = (i, j)
        if u == (-1, -1):
            break  # no more reachable nodes
        r, c = u
        visited[r, c] = True
        if (r, c) == (goal_row, goal_col):
            break

        for dr, dc, step_len in directions:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if visited[nr, nc]:
                continue
            cell_cost = costmap[nr, nc]
            if cell_cost >= lethal_threshold:
                continue
            new_cost = dist[r, c] + step_len + cell_cost
            if new_cost < dist[nr, nc]:
                dist[nr, nc] = new_cost
                parent[nr, nc, 0] = r
                parent[nr, nc, 1] = c

    # Check if goal reached
    if not visited[goal_row, goal_col]:
        return []

    # Reconstruct path (grid indices to world coordinates)
    path_indices = []
    r, c = goal_row, goal_col
    while (r, c) != (start_row, start_col):
        path_indices.append((c, r))   # (x, y)
        pr, pc = parent[r, c, 0], parent[r, c, 1]
        if pr == -1:
            return []   # should not happen if goal reachable
        r, c = pr, pc
    path_indices.append((start_col, start_row))
    path_indices.reverse()

    # Convert to world coordinates (cell centers)
    
    return path_indices
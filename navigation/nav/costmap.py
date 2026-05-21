"""Costmap generation: obstacle inflation and lidar-based dynamic costmap."""

from __future__ import annotations

from typing import Tuple

import numpy as np
import math
from scipy import ndimage

def compute_costmap(
    static_map: np.ndarray,
) -> np.ndarray:
    """
    Build the global costmap by inflating static obstacles.

    Parameters
    ----------
    static_map : np.ndarray, shape (rows, cols), dtype int8
        0 = free cell, 1 = obstacle cell.
    
    Returns
    -------
    costmap : np.ndarray, shape (rows, cols), dtype uint8
        Per-cell cost in [0, 255]:
        - obstacle cells get the maximum lethal value, so the planner
          treats them as impassable.
        - free cells near an obstacle get a non-zero cost that decays with
          distance, creating a "buffer" so the planned path keeps clear of
          walls instead of grazing them.
        - free cells far from any obstacle get cost 0.

    Notes
    -----
    - The classical recipe: compute the Euclidean distance from each free cell
      to the nearest obstacle (`scipy.ndimage.distance_transform_edt` does this
      in one call), then map distance → cost so that distance 0 is lethal and
      cost falls off smoothly out to some `inflation_radius`. Beyond that
      radius, cost should be 0.
    - The shape of the decay (linear, exponential, ...) and the magnitude of
      the inflation radius are tuning knobs. Pick something that visibly biases
      the path away from walls without making narrow passages impassable. The
      inflation radius that is too large will also cause the robot to take a
      longer route, wasting time.
    """
    inflation_radius=3
    rows, cols = static_map.shape
    static_map_reverse=np.ones((rows,cols))-static_map
    distance = ndimage.distance_transform_edt(static_map_reverse)
    costmap = np.where(static_map_reverse == 1, 0, 255).astype(np.uint8)
    mask = (distance > 0) & (distance <= inflation_radius)
    costmap[mask] = (255 * (1 - distance[mask] / inflation_radius)).astype(np.uint8)
    
    return costmap


def update_local_costmap(
    static_map: np.ndarray,
    robot_pos: Tuple[float, float],
    lidar_scan: np.ndarray,
    lidar_range: float,
    lidar_num_rays: int,
) -> np.ndarray:
    """
    Produce the per-frame costmap by adding a dynamic layer on top of the
    static inflation.

    Parameters
    ----------
    static_map : np.ndarray, shape (rows, cols), dtype int8
        The same static map passed to `compute_costmap`. Re-inflate it (or
        cache the result) to get the static layer.
    robot_pos : Tuple[float, float], (x, y)
        Current robot position in world (grid-unit) coordinates. Lidar rays
        originate from this point.
    lidar_scan : np.ndarray, shape (lidar_num_rays,)
        Hit distance for each ray, in grid units. A value equal to `lidar_range`
        means the ray did not hit anything within range.
    lidar_range : float
        Maximum sensing distance of the lidar, in grid units.
    lidar_num_rays : int
        Number of rays in the scan; the i-th ray is at angle
        `2*pi * i / lidar_num_rays` measured from the +x axis.

    Returns
    -------
    costmap : np.ndarray, shape (rows, cols), dtype uint8
        Static-inflation layer merged with a dynamic layer that marks lidar
        hits as lethal and inflates them with a (smaller) buffer. Use a
        per-cell `max` to combine the two layers so the most conservative
        cost wins.

    Notes
    -----
    - Convert each ray hit `(angle_i, lidar_scan[i])` into a world point
      `(x + d*cos(a), y + d*sin(a))`, then to a grid cell. Mark that cell
      lethal and inflate it.
    - Skip rays where `lidar_scan[i] >= lidar_range` (no hit).
    - Optional but useful: skip hits that land on a cell that is *already*
      a static obstacle; otherwise the lidar's view of a wall keeps
      re-inflating the same area.
    """
    rows, cols = static_map.shape
    dyn_map=np.zeros((rows, cols))
    # TODO: Implement a function to update the global costmap with a local dynamic layer based on the lidar scan.
    for i in range(lidar_num_rays):
        if lidar_scan[i] >= lidar_range:
            continue
        angle=2*math.pi*i/lidar_num_rays
        world_point=[0,0]
        world_point[0]=int(robot_pos[0]+lidar_scan[i]*math.cos(angle))
        world_point[1]=int(robot_pos[1]+lidar_scan[i]*math.sin(angle))
        if world_point[0]<0 or world_point[0]>=cols or world_point[1]<0 or world_point[1]>=rows:
            continue
        
        dyn_map[world_point[1],world_point[0]]=1
   
    
    
    from scipy.ndimage import binary_dilation
    dynamic_radius = 1   # 小膨胀半径
    struct = np.ones((2*dynamic_radius+1, 2*dynamic_radius+1))
    dyn_map = binary_dilation(dyn_map, structure=struct)
    dynamic_costmap = np.zeros((rows, cols), dtype=np.uint8)
    dynamic_costmap[dyn_map] = 255
    costmap_static=compute_costmap(static_map)
    cost=np.maximum(dynamic_costmap,costmap_static)
        
        
    return cost






print(compute_costmap(np.array(([0,1,1,1,1],
              [0,0,1,1,1],
              [0,1,1,1,1],
              [0,1,1,1,0],
              [0,1,1,0,0]))))
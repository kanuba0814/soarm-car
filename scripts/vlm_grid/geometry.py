"""Conservative per-frame RGB-D occupancy and route primitives."""
from collections import deque
import math

import numpy as np


UNKNOWN = 0
FREE = 1
OCCUPIED = 2


def world_to_cell(point, origin, resolution):
    return tuple(np.floor((np.asarray(point, dtype=float) - origin) / resolution).astype(int))


def cell_to_world(cell, origin, resolution):
    return origin + (np.asarray(cell, dtype=float) + .5) * resolution


def line_cells(first, second):
    """Integer Bresenham cells including both endpoints."""
    x0, y0 = first
    x1, y1 = second
    cells = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx + dy
    while True:
        cells.append((x0, y0))
        if (x0, y0) == (x1, y1):
            return cells
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += sx
        if twice_error <= dx:
            error += dx
            y0 += sy


def inflate_occupied(grid, radius_cells):
    inflated = grid.copy()
    for row, column in np.argwhere(grid == OCCUPIED):
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc <= radius_cells * radius_cells:
                    target = row + dr, column + dc
                    if 0 <= target[0] < grid.shape[0] and 0 <= target[1] < grid.shape[1]:
                        inflated[target] = OCCUPIED
    return inflated


def build_grid(sensor_xy, endpoints_xyz, endpoint_is_obstacle, resolution, robot_radius_m):
    """Raycast observed free space; unknown remains non-free by construction."""
    endpoints_xy = np.asarray(endpoints_xyz, dtype=float)[:, :2]
    all_points = np.vstack((np.asarray(sensor_xy, dtype=float), endpoints_xy))
    lower = np.floor(all_points.min(axis=0) / resolution) * resolution - robot_radius_m - resolution
    upper = np.ceil(all_points.max(axis=0) / resolution) * resolution + robot_radius_m + resolution
    shape = tuple(np.ceil((upper - lower) / resolution).astype(int)[::-1])
    grid = np.full(shape, UNKNOWN, dtype=np.uint8)
    source = world_to_cell(sensor_xy, lower, resolution)
    for endpoint, obstacle in zip(endpoints_xy, endpoint_is_obstacle):
        destination = world_to_cell(endpoint, lower, resolution)
        cells = line_cells(source, destination)
        for x, y in cells[:-1]:
            if 0 <= y < shape[0] and 0 <= x < shape[1] and grid[y, x] != OCCUPIED:
                grid[y, x] = FREE
        x, y = cells[-1]
        if 0 <= y < shape[0] and 0 <= x < shape[1]:
            grid[y, x] = OCCUPIED if obstacle else FREE
    return grid, lower


def astar(grid, start, goal):
    if not (0 <= start[0] < grid.shape[1] and 0 <= start[1] < grid.shape[0] and 0 <= goal[0] < grid.shape[1] and 0 <= goal[1] < grid.shape[0]):
        return None, "outside_grid"
    if grid[start[1], start[0]] != FREE:
        return None, "start_not_observed_free"
    if grid[goal[1], goal[0]] != FREE:
        return None, "goal_occupied_or_unknown"
    queue = deque([start])
    parent = {start: None}
    while queue:
        current = queue.popleft()
        if current == goal:
            route = []
            while current is not None:
                route.append(current)
                current = parent[current]
            return route[::-1], None
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            successor = current[0] + dx, current[1] + dy
            if successor in parent or not (0 <= successor[0] < grid.shape[1] and 0 <= successor[1] < grid.shape[0]):
                continue
            if grid[successor[1], successor[0]] == FREE:
                parent[successor] = current
                queue.append(successor)
    return None, "no_free_route"


def reachable_approach(grid, origin, resolution, start_xy, target_xy, approach_radius_m):
    """Find the shortest free route to a ring around, never the target centroid."""
    start = world_to_cell(start_xy, origin, resolution)
    proposals = []
    for index in range(24):
        angle = 2 * math.pi * index / 24
        point = np.asarray(target_xy) + approach_radius_m * np.array([math.cos(angle), math.sin(angle)])
        route, reason = astar(grid, start, world_to_cell(point, origin, resolution))
        if route:
            proposals.append((len(route), point, route))
    if not proposals:
        return None, "all_approach_points_occupied_or_unknown"
    _, point, route = min(proposals, key=lambda item: item[0])
    return {"approach_xy_m": point.tolist(), "route_cells": [[int(value) for value in cell] for cell in route], "route_length_m": (len(route) - 1) * resolution}, None

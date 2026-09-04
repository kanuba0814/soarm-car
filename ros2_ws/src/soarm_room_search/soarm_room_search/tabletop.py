"""Conservative dependency-free tabletop baseline; input points are already in map frame."""
import math
import struct
from collections import defaultdict, deque

def decode_xyz(data, point_step=12, x_offset=0, y_offset=4, z_offset=8, limit=200000):
    points = []
    for offset in range(0, min(len(data), point_step * limit), point_step):
        x, y, z = struct.unpack_from('<fff', data, offset + x_offset)
        if all(math.isfinite(value) for value in (x, y, z)):
            points.append((x, y, z))
    return points

def detect_tabletops(points, min_height=.55, max_height=1.15, voxel=.08, min_points=20, min_size=.45, max_size=2.5):
    """Cluster horizontal-height support cells; returns fail-closed desk hypotheses."""
    cells = defaultdict(list)
    for x, y, z in points:
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if min_height <= z <= max_height:
            cells[(math.floor(x / voxel), math.floor(y / voxel))].append((x, y, z))
    seen, desks = set(), []
    for start in sorted(cells):
        if start in seen:
            continue
        queue, group = deque([start]), []
        seen.add(start)
        while queue:
            cell = queue.popleft(); group.extend(cells[cell])
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                neighbor = (cell[0]+dx, cell[1]+dy)
                if neighbor in cells and neighbor not in seen:
                    seen.add(neighbor); queue.append(neighbor)
        if len(group) < min_points:
            continue
        xs, ys, zs = zip(*group); width, depth = max(xs)-min(xs), max(ys)-min(ys)
        if not (min_size <= width <= max_size and min_size <= depth <= max_size):
            continue
        mean = sum(zs)/len(zs); spread = sum((z-mean)**2 for z in zs)/len(zs)
        if spread > .015**2:
            continue
        x, y = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2
        yaw = 0. if width >= depth else math.pi/2
        quality = min(1., len(group)/(min_points*3)) * max(0., 1.-spread/.015**2)
        if quality < .6:
            continue
        docks = []
        for sign in (-1, 1):
            distance = max(width, depth)/2 + .65
            dx, dy = math.cos(yaw)*distance*sign, math.sin(yaw)*distance*sign
            docks.append({'dock': (x+dx,y+dy), 'dock_yaw': math.atan2(-dy,-dx)})
        desks.append({'x':x,'y':y,'height':mean,'width':width,'depth':depth,'yaw':yaw,'quality':quality,'uncertainty':math.sqrt(spread),'docks':docks,'line_of_sight':True})
    return desks

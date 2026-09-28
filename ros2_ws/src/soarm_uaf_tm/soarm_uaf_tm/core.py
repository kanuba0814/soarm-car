"""Dependency-light UAF-TM temporal occupancy core."""
import math

class TemporalMap:
    def __init__(self, resolution=0.10, decay=0.995):
        self.resolution, self.decay = resolution, decay
        self.cells = {}  # (x, y): confidence in [0, 1]

    def update(self, points, pose=(0.0, 0.0, 0.0), confidence=1.0):
        for key in list(self.cells):
            self.cells[key] *= self.decay
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        for x, y in points:
            wx, wy = x0 + c * x - s * y, y0 + s * x + c * y
            key = (math.floor(wx / self.resolution), math.floor(wy / self.resolution))
            self.cells[key] = min(1.0, self.cells.get(key, 0.0) + 0.35 * confidence)
        return self.cells

    def occupied(self, threshold=0.5):
        return {key for key, value in self.cells.items() if value >= threshold}

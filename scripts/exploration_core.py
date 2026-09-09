#!/usr/bin/env python3
"""Deterministic, CPU-only prior-map-assisted frontier selection.

This module deliberately does not import ROS or use wall-clock time.  The caller
owns sensor freshness and Nav2 ComputePathToPose execution.
"""
import argparse
import hashlib
import json
import math
import time
from collections import deque
from pathlib import Path

ROBOT_RADIUS_M = math.sqrt(.3 ** 2 + .3 ** 2) + .05

class InputError(ValueError):
    pass

class PriorMap:
    def __init__(self, width, height, resolution, origin, pixels, map_identity, free_threshold=.25, occupied_threshold=.65):
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
            raise InputError("invalid map origin")
        if abs(origin[2]) > 1e-12:
            raise InputError("nonzero origin yaw is not supported")
        self.width, self.height = width, height
        self.resolution, self.origin = resolution, tuple(origin)
        self.pixels, self.map_identity = pixels, map_identity
        self.free_threshold, self.occupied_threshold = free_threshold, occupied_threshold

    @classmethod
    def from_yaml(cls, yaml_path, map_identity):
        values = {}
        for line in Path(yaml_path).read_text().splitlines():
            if ":" in line and not line.lstrip().startswith("#"):
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip().strip("'\"")
        image_path = Path(yaml_path).parent / values["image"]
        width, height, pixels = _load_pgm(image_path)
        origin = tuple(float(value.strip()) for value in values["origin"].strip("[]").split(","))
        return cls(width, height, float(values["resolution"]), origin, pixels, map_identity,
                   float(values.get("free_thresh", ".25")), float(values.get("occupied_thresh", ".65")))

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def world_to_cell(self, point):
        return (math.floor((point[0] - self.origin[0]) / self.resolution),
                math.floor((point[1] - self.origin[1]) / self.resolution))

    def cell_center(self, cell):
        return (self.origin[0] + (cell[0] + .5) * self.resolution,
                self.origin[1] + (cell[1] + .5) * self.resolution)

    def occupancy(self, cell):
        if not self.inside(cell):
            return "unknown"
        row = self.height - 1 - cell[1]
        occupied = (255 - self.pixels[row * self.width + cell[0]]) / 255.0
        if occupied >= self.occupied_threshold:
            return "occupied"
        if occupied <= self.free_threshold:
            return "free"
        return "unknown"

def _load_pgm(path):
    raw, tokens, offset = Path(path).read_bytes(), [], 0
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
        if offset >= len(raw): raise InputError("truncated PGM")
        if raw[offset:offset + 1] == b"#":
            offset = raw.find(b"\n", offset) + 1
            continue
        end = offset
        while end < len(raw) and not raw[end:end + 1].isspace(): end += 1
        tokens.append(raw[offset:end]); offset = end
    if tokens[0] != b"P5" or tokens[3] != b"255": raise InputError("expected binary 8-bit PGM")
    while offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
    width, height = int(tokens[1]), int(tokens[2]); pixels = raw[offset:]
    if len(pixels) != width * height: raise InputError("PGM pixel count mismatch")
    return width, height, pixels

def load_pinned_prior(manifest_path):
    manifest = json.loads(Path(manifest_path).read_text())
    base = Path(manifest_path).parent
    yaml_path = (base / manifest["yaml_path"]).resolve()
    image_path = (base / manifest["pgm_path"]).resolve()
    if hashlib.sha256(image_path.read_bytes()).hexdigest() != manifest["pgm_sha256"]:
        raise InputError("pinned PGM digest mismatch")
    prior = PriorMap.from_yaml(yaml_path, manifest["map_identity"])
    expected = manifest["geometry"]
    if [prior.width, prior.height] != expected["dimensions"] or prior.resolution != expected["resolution"] or list(prior.origin) != expected["origin"]:
        raise InputError("pinned map geometry mismatch")
    return prior

def _finite_point(value, fields=2):
    return isinstance(value, (list, tuple)) and len(value) >= fields and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value[:fields])

def _line_cells(prior, start, end, limit=4096):
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = min(limit, max(1, math.ceil(distance / (prior.resolution * .5))))
    previous = None
    for index in range(count + 1):
        cell = prior.world_to_cell((start[0] + (end[0] - start[0]) * index / count, start[1] + (end[1] - start[1]) * index / count))
        if cell != previous:
            yield cell
            previous = cell

def _neighbors(cell):
    for delta in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        yield (cell[0] + delta[0], cell[1] + delta[1])

class ExplorationCore:
    """Stateful observation fusion and deterministic frontier shortlist API."""
    def __init__(self, prior, max_rays=2048, max_candidates=16, max_attempts=2):
        self.prior = prior
        self.max_rays, self.max_candidates, self.max_attempts = max_rays, max_candidates, max_attempts
        self.observed, self.live_obstacles = set(), set()
        self.revision = 0
        self.cluster_attempts, self.cluster_last_gain = {}, {}
        self._accepted_ids = set()

    def apply_observation(self, record):
        required = ("id", "revision", "map_identity", "frame", "sensor_origin", "robot_pose", "returns")
        if not isinstance(record, dict) or any(key not in record for key in required): raise InputError("observation fields missing")
        if record["map_identity"] != self.prior.map_identity: raise InputError("wrong map identity")
        if not isinstance(record["id"], str) or not record["id"]: raise InputError("invalid observation id")
        if record["frame"] != "map" or not _finite_point(record["sensor_origin"], 3) or not _finite_point(record["robot_pose"], 3): raise InputError("invalid frame or pose")
        if not isinstance(record["revision"], int) or record["revision"] <= 0: raise InputError("invalid revision")
        if record["id"] in self._accepted_ids: return {"status": "duplicate", "revision": self.revision, "new_observed_cells": 0}
        if record["revision"] <= self.revision: raise InputError("stale revision")
        if not self.prior.inside(self.prior.world_to_cell(record["sensor_origin"])): raise InputError("sensor origin out of bounds")
        returns = record["returns"]
        if not isinstance(returns, list): raise InputError("returns must be a list")
        pending_observed, pending_obstacles = set(), set()
        for item in returns[:self.max_rays]:
            if not isinstance(item, dict) or item.get("kind") == "no_return": continue
            point = item.get("xyz")
            if not _finite_point(point, 3): raise InputError("nonfinite measured return")
            # A finite point supplies visibility only. It does not override prior unknowns or certify body clearance.
            for cell in _line_cells(self.prior, record["sensor_origin"], point):
                if not self.prior.inside(cell): break
                pending_observed.add(cell)
                if self.prior.occupancy(cell) == "occupied": break
                if cell == self.prior.world_to_cell(point):
                    if item.get("kind") == "obstacle": pending_obstacles.add(cell)
                    break
        before = len(self.observed)
        self.observed.update(pending_observed)
        self.live_obstacles.update(pending_obstacles)  # Occupied evidence wins irrespective of record order.
        self.revision = record["revision"]
        self._accepted_ids.add(record["id"])
        return {"status": "applied", "revision": self.revision, "new_observed_cells": len(self.observed) - before}

    def _footprint_ok(self, cell):
        center = self.prior.cell_center(cell)
        extent = ROBOT_RADIUS_M
        minimum_x = math.floor((center[0] - extent - self.prior.origin[0]) / self.prior.resolution)
        maximum_x = math.floor((center[0] + extent - self.prior.origin[0]) / self.prior.resolution)
        minimum_y = math.floor((center[1] - extent - self.prior.origin[1]) / self.prior.resolution)
        maximum_y = math.floor((center[1] + extent - self.prior.origin[1]) / self.prior.resolution)
        for x in range(minimum_x, maximum_x + 1):
            for y in range(minimum_y, maximum_y + 1):
                candidate = (x, y)
                left, bottom = self.prior.cell_center(candidate)[0] - self.prior.resolution / 2, self.prior.cell_center(candidate)[1] - self.prior.resolution / 2
                nearest_x, nearest_y = max(left, min(center[0], left + self.prior.resolution)), max(bottom, min(center[1], bottom + self.prior.resolution))
                # Circle/cell intersection catches square-corner collisions at (.275, .275).
                if math.hypot(nearest_x - center[0], nearest_y - center[1]) <= extent:
                    if not self.prior.inside(candidate) or candidate not in self.observed or candidate in self.live_obstacles or self.prior.occupancy(candidate) != "free": return False
        return True

    def _traversable(self, cell):
        return cell in self.observed and cell not in self.live_obstacles and self.prior.occupancy(cell) == "free" and self._footprint_ok(cell)

    def _reachable(self, start):
        if not self._traversable(start): return set()
        seen, queue = {start}, deque([start])
        while queue:
            cell = queue.popleft()
            for adjacent in _neighbors(cell):
                if adjacent not in seen and self._traversable(adjacent): seen.add(adjacent); queue.append(adjacent)
        return seen

    def candidates(self, robot_pose):
        if not _finite_point(robot_pose, 3): raise InputError("invalid robot pose")
        start = self.prior.world_to_cell(robot_pose)
        reachable = self._reachable(start)
        # A frontier is a visible, prior-traversable boundary cell. Its footprint is
        # intentionally not required to fit: the selected approach is offset inward.
        frontier = {cell for cell in self.observed if cell not in self.live_obstacles and self.prior.occupancy(cell) == "free" and any(adjacent not in self.observed for adjacent in _neighbors(cell))}
        if not frontier:
            reason = "no_frontier" if any(self._traversable(cell) for cell in self.observed) else "blocked"
            return {"status": reason, "revision": self.revision, "candidates": [], "observed_area_cells": len(self.observed), "reference_prior_connected_area_cells": self._prior_connected_area(start)}
        groups = _clusters(frontier)
        output = []
        for group in groups:
            cluster_id = _cluster_id(group)
            if self.cluster_attempts.get(cluster_id, 0) >= self.max_attempts: continue
            offset_cells = math.ceil(ROBOT_RADIUS_M / self.prior.resolution)
            # Generate a bounded inward ring rather than comparing every reachable
            # configuration cell to every frontier cell.
            approaches = set()
            for frontier_cell in group:
                for dx in range(-offset_cells - 1, offset_cells + 2):
                    for dy in range(-offset_cells - 1, offset_cells + 2):
                        if offset_cells <= abs(dx) + abs(dy) <= offset_cells + 1:
                            candidate = (frontier_cell[0] + dx, frontier_cell[1] + dy)
                            if candidate in reachable: approaches.add(candidate)
            if not approaches: continue
            approach = min(approaches, key=lambda cell: (math.hypot(cell[0] - start[0], cell[1] - start[1]), cell))
            gain = sum(1 for cell in group for n in _neighbors(cell) if n not in self.observed)
            output.append({"cluster_id": cluster_id, "goal": list(self.prior.cell_center(approach)), "frontier_cells": len(group), "predicted_gain_cells": gain, "distance_m": round(math.hypot(approach[0] - start[0], approach[1] - start[1]) * self.prior.resolution, 6)})
        output.sort(key=lambda item: (-item["predicted_gain_cells"], item["distance_m"], item["cluster_id"]))
        status = "ok" if output else "exhausted"
        return {"status": status, "revision": self.revision, "candidates": output[:self.max_candidates], "observed_area_cells": len(self.observed), "reference_prior_connected_area_cells": self._prior_connected_area(start)}

    def report_result(self, cluster_id, gained_observed_cells, succeeded):
        if not isinstance(gained_observed_cells, int) or gained_observed_cells < 0: raise InputError("invalid gain")
        if not succeeded or gained_observed_cells == 0: self.cluster_attempts[cluster_id] = self.cluster_attempts.get(cluster_id, 0) + 1
        self.cluster_last_gain[cluster_id] = gained_observed_cells

    def _prior_connected_area(self, start):
        if self.prior.occupancy(start) != "free": return 0
        seen, queue = {start}, deque([start])
        while queue:
            cell = queue.popleft()
            for adjacent in _neighbors(cell):
                if adjacent not in seen and self.prior.occupancy(adjacent) == "free": seen.add(adjacent); queue.append(adjacent)
        return len(seen)

def _clusters(cells):
    remaining, groups = set(cells), []
    while remaining:
        seed, group, queue = min(remaining), set(), deque([min(remaining)])
        remaining.remove(seed)
        while queue:
            cell = queue.popleft(); group.add(cell)
            for adjacent in _neighbors(cell):
                if adjacent in remaining: remaining.remove(adjacent); queue.append(adjacent)
        groups.append(group)
    return groups

def _cluster_id(group):
    digest = hashlib.sha256(",".join("%d:%d" % cell for cell in sorted(group)).encode()).hexdigest()[:16]
    return "frontier-" + digest

def run_cli(input_path, output_path):
    payload = json.loads(Path(input_path).read_text())
    prior = load_pinned_prior(payload["prior_manifest"])
    core = ExplorationCore(prior, **payload.get("config", {}))
    started_cpu = time.process_time()
    results = [core.apply_observation(record) for record in payload.get("observations", [])]
    shortlist = core.candidates(payload["robot_pose"])
    output = {"observation_results": results, "shortlist": shortlist, "runtime": {"cpu_only": True, "cpu_process_seconds": round(time.process_time() - started_cpu, 6), "estimated_grid_memory_bytes": prior.width * prior.height * 3}}
    Path(output_path).write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    return output

def main():
    parser = argparse.ArgumentParser(); parser.add_argument("input"); parser.add_argument("output")
    args = parser.parse_args(); run_cli(args.input, args.output)
if __name__ == "__main__": main()

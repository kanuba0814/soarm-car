#!/usr/bin/env python3
"""Deterministic, CPU-only prior-map-assisted frontier selection."""
import argparse
import hashlib
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np

ROBOT_RADIUS_M = math.sqrt(.3 ** 2 + .3 ** 2) + .05
NAV2_ARRIVAL_TOLERANCE_M = .2

class InputError(ValueError):
    pass

class PriorMap:
    def __init__(self, width, height, resolution, origin, pixels, map_identity, free_threshold=.25, occupied_threshold=.65):
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin): raise InputError("invalid map origin")
        if abs(origin[2]) > 1e-12: raise InputError("nonzero origin yaw is not supported")
        self.width, self.height, self.resolution, self.origin = width, height, resolution, tuple(origin)
        self.pixels, self.map_identity = pixels, map_identity
        self.free_threshold, self.occupied_threshold = free_threshold, occupied_threshold

    @classmethod
    def from_yaml(cls, yaml_path, map_identity):
        values = {}
        for line in Path(yaml_path).read_text().splitlines():
            if ":" in line and not line.lstrip().startswith("#"):
                key, value = line.split(":", 1); values[key.strip()] = value.strip().strip("'\"")
        image_path = Path(yaml_path).parent / values["image"]
        width, height, pixels = _load_pgm(image_path)
        origin = tuple(float(value.strip()) for value in values["origin"].strip("[]").split(","))
        return cls(width, height, float(values["resolution"]), origin, pixels, map_identity, float(values.get("free_thresh", ".25")), float(values.get("occupied_thresh", ".65")))

    def inside(self, cell): return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height
    def world_to_cell(self, point): return (math.floor((point[0] - self.origin[0]) / self.resolution), math.floor((point[1] - self.origin[1]) / self.resolution))
    def cell_center(self, cell): return (self.origin[0] + (cell[0] + .5) * self.resolution, self.origin[1] + (cell[1] + .5) * self.resolution)
    def occupancy(self, cell):
        if not self.inside(cell): return "unknown"
        pixel = self.pixels[(self.height - 1 - cell[1]) * self.width + cell[0]]
        occupied = (255 - pixel) / 255.0
        if occupied >= self.occupied_threshold: return "occupied"
        if occupied <= self.free_threshold: return "free"
        return "unknown"

def _load_pgm(path):
    raw, tokens, offset = Path(path).read_bytes(), [], 0
    while len(tokens) < 4:
        while offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
        if offset >= len(raw): raise InputError("truncated PGM")
        if raw[offset:offset + 1] == b"#": offset = raw.find(b"\n", offset) + 1; continue
        end = offset
        while end < len(raw) and not raw[end:end + 1].isspace(): end += 1
        tokens.append(raw[offset:end]); offset = end
    if tokens[0] != b"P5" or tokens[3] != b"255": raise InputError("expected binary 8-bit PGM")
    while offset < len(raw) and raw[offset:offset + 1].isspace(): offset += 1
    width, height, pixels = int(tokens[1]), int(tokens[2]), raw[offset:]
    if len(pixels) != width * height: raise InputError("PGM pixel count mismatch")
    return width, height, pixels

def load_pinned_prior(manifest_path):
    manifest = json.loads(Path(manifest_path).read_text()); base = Path(manifest_path).parent
    yaml_path, image_path = (base / manifest["yaml_path"]).resolve(), (base / manifest["pgm_path"]).resolve()
    if hashlib.sha256(image_path.read_bytes()).hexdigest() != manifest["pgm_sha256"]: raise InputError("pinned PGM digest mismatch")
    prior = PriorMap.from_yaml(yaml_path, manifest["map_identity"]); expected = manifest["geometry"]
    if [prior.width, prior.height] != expected["dimensions"] or prior.resolution != expected["resolution"] or list(prior.origin) != expected["origin"]: raise InputError("pinned map geometry mismatch")
    return prior

def _finite_point(value, fields=2): return isinstance(value, (list, tuple)) and len(value) >= fields and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value[:fields])
def _neighbors(cell):
    for delta in ((1, 0), (-1, 0), (0, 1), (0, -1)): yield (cell[0] + delta[0], cell[1] + delta[1])
def _manhattan(a, b): return abs(a[0] - b[0]) + abs(a[1] - b[1])
def _cells_to_json(cells): return [list(cell) for cell in sorted(cells)]
def _cells_from_json(cells): return {tuple(cell) for cell in cells}

def _line_cells(prior, start, end, limit=4096):
    distance = math.hypot(end[0] - start[0], end[1] - start[1]); count = min(limit, max(1, math.ceil(distance / (prior.resolution * .5))))
    previous = None
    for index in range(count + 1):
        cell = prior.world_to_cell((start[0] + (end[0] - start[0]) * index / count, start[1] + (end[1] - start[1]) * index / count))
        if cell != previous: yield cell; previous = cell

def _clusters(cells):
    remaining, groups = set(cells), []
    while remaining:
        seed, group, queue = min(remaining), set(), deque([min(remaining)]); remaining.remove(seed)
        while queue:
            cell = queue.popleft(); group.add(cell)
            for adjacent in _neighbors(cell):
                if adjacent in remaining: remaining.remove(adjacent); queue.append(adjacent)
        groups.append(group)
    return groups

def _cluster_id(group): return "frontier-" + hashlib.sha256(",".join("%d:%d" % cell for cell in sorted(group)).encode()).hexdigest()[:16]

class ExplorationCore:
    """Transport-neutral frontier core. `observed` is navigation-band evidence only."""
    STATE_VERSION = 2
    def __init__(self, prior, max_rays=2048, max_candidates=16, max_attempts=2, navigation_z_min=-.30, navigation_z_max=.50):
        if not (0 < max_rays <= 2048 and 0 < max_candidates <= 128 and 0 < max_attempts <= 16): raise InputError("invalid bounded configuration")
        if not all(math.isfinite(v) for v in (navigation_z_min, navigation_z_max)) or navigation_z_min > navigation_z_max: raise InputError("invalid navigation height band")
        self.prior, self.max_rays, self.max_candidates, self.max_attempts = prior, max_rays, max_candidates, max_attempts
        self.navigation_z_min, self.navigation_z_max = navigation_z_min, navigation_z_max
        self.observed, self.visibility, self.live_obstacles = set(), set(), set()
        self.revision, self._accepted_ids, self._retry_regions, self._issued_regions = 0, set(), [], {}
        self._prior_free = np.array([[prior.occupancy((x, y)) == 'free' for y in range(prior.height)] for x in range(prior.width)], dtype=bool)
        extent_cells = math.ceil(ROBOT_RADIUS_M / prior.resolution)
        self._footprint_offsets = tuple((dx, dy) for dx in range(-extent_cells, extent_cells + 1) for dy in range(-extent_cells, extent_cells + 1) if self._offset_intersects_footprint(dx, dy))
        self._configuration_cache_revision = None
        self._configuration_cache = None

    def _offset_intersects_footprint(self, dx, dy):
        half = self.prior.resolution / 2
        nearest_x = max(abs(dx) * self.prior.resolution - half, 0.0)
        nearest_y = max(abs(dy) * self.prior.resolution - half, 0.0)
        return math.hypot(nearest_x, nearest_y) <= ROBOT_RADIUS_M

    def _configuration_space(self):
        if self._configuration_cache_revision == self.revision and self._configuration_cache is not None:
            return self._configuration_cache
        base = self._prior_free.copy()
        for x, y in self.observed:
            base[x, y] = True
        # Observation is the authoritative free-space evidence; prior free cells alone are insufficient.
        observed = np.zeros_like(base)
        for x, y in self.observed:
            observed[x, y] = True
        base = observed & self._prior_free
        for x, y in self.live_obstacles:
            base[x, y] = False
        configuration = base.copy()
        width, height = self.prior.width, self.prior.height
        for dx, dy in self._footprint_offsets:
            shifted = np.zeros_like(base)
            source_x0, source_x1 = max(0, dx), min(width, width + dx)
            source_y0, source_y1 = max(0, dy), min(height, height + dy)
            target_x0, target_x1 = max(0, -dx), min(width, width - dx)
            target_y0, target_y1 = max(0, -dy), min(height, height - dy)
            if target_x0 < target_x1 and target_y0 < target_y1:
                shifted[target_x0:target_x1, target_y0:target_y1] = base[source_x0:source_x1, source_y0:source_y1]
            configuration &= shifted
        self._configuration_cache_revision, self._configuration_cache = self.revision, configuration
        return configuration

    def _in_navigation_band(self, point, ground_origin, ground_normal):
        height = sum((point[index] - ground_origin[index]) * ground_normal[index] for index in range(3))
        return self.navigation_z_min <= height <= self.navigation_z_max

    def _nav_return(self, origin, item, endpoint, ground_origin, ground_normal):
        return (item.get("kind") in ("hit", "obstacle") and
                self._in_navigation_band(origin, ground_origin, ground_normal) and
                self._in_navigation_band(endpoint, ground_origin, ground_normal))

    def apply_observation(self, record):
        required = ("id", "revision", "map_identity", "frame", "sensor_origin", "robot_pose", "returns")
        if not isinstance(record, dict) or any(key not in record for key in required): raise InputError("observation fields missing")
        if record["map_identity"] != self.prior.map_identity: raise InputError("wrong map identity")
        if not isinstance(record["id"], str) or not record["id"]: raise InputError("invalid observation id")
        if record["frame"] != "map" or not _finite_point(record["sensor_origin"], 3) or not _finite_point(record["robot_pose"], 3): raise InputError("invalid frame or pose")
        ground_reference = record.get("ground_reference_z", 0.0)
        if not isinstance(ground_reference, (int, float)) or not math.isfinite(ground_reference): raise InputError("invalid ground reference")
        ground_origin = record.get("ground_plane_origin", [0.0, 0.0, ground_reference])
        ground_normal = record.get("ground_plane_normal", [0.0, 0.0, 1.0])
        if not _finite_point(ground_origin, 3) or not _finite_point(ground_normal, 3): raise InputError("invalid ground plane")
        normal_length = math.sqrt(sum(value * value for value in ground_normal))
        if abs(normal_length - 1.0) > 1e-6: raise InputError("ground plane normal must be unit")
        if not isinstance(record["revision"], int) or record["revision"] <= 0: raise InputError("invalid revision")
        if record["id"] in self._accepted_ids: return {"status": "duplicate", "revision": self.revision, "new_observed_cells": 0}
        if record["revision"] <= self.revision: raise InputError("stale revision")
        origin, returns = record["sensor_origin"], record["returns"]
        if not self.prior.inside(self.prior.world_to_cell(origin)): raise InputError("sensor origin out of bounds")
        if not isinstance(returns, list): raise InputError("returns must be a list")
        parsed = []
        for item in returns[:self.max_rays]:
            if not isinstance(item, dict) or item.get("kind") == "no_return": continue
            endpoint = item.get("xyz")
            if not _finite_point(endpoint, 3): raise InputError("nonfinite measured return")
            parsed.append((item, endpoint, self.prior.world_to_cell(endpoint)))
        # First pass makes all same-record obstacle endpoints opaque before any ray is traced.
        blockers = set(self.live_obstacles)
        blockers.update(cell for item, endpoint, cell in parsed if item.get("kind") == "obstacle" and self._nav_return(origin, item, endpoint, ground_origin, ground_normal) and self.prior.inside(cell))
        pending_visibility, pending_navigation = set(), set()
        for item, endpoint, endpoint_cell in parsed:
            navigation_evidence = self._nav_return(origin, item, endpoint, ground_origin, ground_normal)
            for cell in _line_cells(self.prior, origin, endpoint):
                if not self.prior.inside(cell): break
                pending_visibility.add(cell)
                if navigation_evidence: pending_navigation.add(cell)
                if self.prior.occupancy(cell) == "occupied" or cell in blockers: break
                if cell == endpoint_cell: break
        before_observed, before_visibility = len(self.observed), len(self.visibility)
        self.visibility.update(pending_visibility); self.observed.update(pending_navigation); self.live_obstacles.update(blockers)
        self.revision = record["revision"]; self._accepted_ids.add(record["id"])
        self._configuration_cache_revision, self._configuration_cache = None, None
        return {"status": "applied", "revision": self.revision, "new_observed_cells": len(self.observed) - before_observed, "new_visibility_cells": len(self.visibility) - before_visibility}

    def _footprint_ok(self, cell):
        center, extent = self.prior.cell_center(cell), ROBOT_RADIUS_M
        min_x, max_x = math.floor((center[0] - extent - self.prior.origin[0]) / self.prior.resolution), math.floor((center[0] + extent - self.prior.origin[0]) / self.prior.resolution)
        min_y, max_y = math.floor((center[1] - extent - self.prior.origin[1]) / self.prior.resolution), math.floor((center[1] + extent - self.prior.origin[1]) / self.prior.resolution)
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                candidate = (x, y); cx, cy = self.prior.cell_center(candidate)
                left, bottom = cx - self.prior.resolution / 2, cy - self.prior.resolution / 2
                nearest_x, nearest_y = max(left, min(center[0], left + self.prior.resolution)), max(bottom, min(center[1], bottom + self.prior.resolution))
                if math.hypot(nearest_x - center[0], nearest_y - center[1]) <= extent:
                    if not self.prior.inside(candidate) or candidate not in self.observed or candidate in self.live_obstacles or self.prior.occupancy(candidate) != "free": return False
        return True

    def _traversable(self, cell):
        return self.prior.inside(cell) and bool(self._configuration_space()[cell[0], cell[1]])
    def _reachable(self, start):
        if not self._traversable(start): return set()
        seen, queue = {start}, deque([start])
        while queue:
            cell = queue.popleft()
            for adjacent in _neighbors(cell):
                if adjacent not in seen and self._traversable(adjacent): seen.add(adjacent); queue.append(adjacent)
        return seen
    def _region_for(self, group):
        tolerance = math.ceil(ROBOT_RADIUS_M / self.prior.resolution) + 1
        matches = [region for region in self._retry_regions if any(_manhattan(cell, old) <= tolerance for cell in group for old in region["cells"])]
        if not matches: return None
        return min(matches, key=lambda region: region["id"])

    def candidates(self, robot_pose):
        if not _finite_point(robot_pose, 3): raise InputError("invalid robot pose")
        start, reachable = self.prior.world_to_cell(robot_pose), self._reachable(self.prior.world_to_cell(robot_pose))
        def eligible_unknown(cell):
            return (self.prior.inside(cell) and self.prior.occupancy(cell) == "free" and
                    cell not in self.observed and cell not in self.live_obstacles)
        frontier = {cell for cell in self.observed if cell not in self.live_obstacles and self.prior.occupancy(cell) == "free" and any(eligible_unknown(adjacent) for adjacent in _neighbors(cell))}
        base = {"revision": self.revision, "observed_area_cells": len(self.observed), "visibility_area_cells": len(self.visibility), "reference_prior_connected_area_cells": self._prior_connected_area(start)}
        if not frontier:
            return dict(base, status="no_frontier" if reachable else "blocked", candidates=[], rejections=[])
        output, rejections, offset = [], [], math.ceil(ROBOT_RADIUS_M / self.prior.resolution)
        for group in _clusters(frontier):
            region, default_id = self._region_for(group), _cluster_id(group)
            cluster_id = region["id"] if region else default_id
            if region and region["attempts"] >= self.max_attempts:
                rejections.append({"cluster_id": cluster_id, "reason": "retry_exhausted"}); continue
            approaches = set()
            for edge in group:
                for dx in range(-offset - 1, offset + 2):
                    for dy in range(-offset - 1, offset + 2):
                        if offset <= abs(dx) + abs(dy) <= offset + 1 and (edge[0] + dx, edge[1] + dy) in reachable: approaches.add((edge[0] + dx, edge[1] + dy))
            if not approaches:
                rejections.append({"cluster_id": cluster_id, "reason": "no_safe_approach"}); continue
            self._issued_regions[cluster_id] = set(group)
            approach = min(approaches, key=lambda cell: (math.hypot(cell[0] - start[0], cell[1] - start[1]), cell))
            distance_m = math.hypot(approach[0] - start[0], approach[1] - start[1]) * self.prior.resolution
            if distance_m < NAV2_ARRIVAL_TOLERANCE_M + self.prior.resolution:
                rejections.append({"cluster_id": cluster_id, "reason": "no_meaningful_approach"}); continue
            gain = len({neighbor for cell in group for neighbor in _neighbors(cell) if eligible_unknown(neighbor)})
            output.append({"cluster_id": cluster_id, "goal": list(self.prior.cell_center(approach)), "frontier_cells": len(group), "predicted_gain_cells": gain, "distance_m": round(distance_m, 6)})
        output.sort(key=lambda item: (-item["predicted_gain_cells"], item["distance_m"], item["cluster_id"]))
        status = "ok" if output else ("exhausted" if rejections and all(item["reason"] == "retry_exhausted" for item in rejections) else "blocked")
        return dict(base, status=status, candidates=output[:self.max_candidates], rejections=rejections)

    def report_result(self, cluster_id, gained_observed_cells, succeeded):
        if not isinstance(cluster_id, str) or not isinstance(gained_observed_cells, int) or gained_observed_cells < 0: raise InputError("invalid result")
        if succeeded and gained_observed_cells > 0: return
        cells = self._issued_regions.get(cluster_id)
        if cells is None: raise InputError("unknown candidate cluster")
        region = self._region_for(cells)
        if region is None:
            self._retry_regions.append({"id": cluster_id, "cells": set(cells), "attempts": 1})
        else:
            region["attempts"] += 1; region["cells"].update(cells)

    def export_state(self):
        """JSON-safe deterministic checkpoint; map frame and ID history are retained."""
        return {"state_version": self.STATE_VERSION, "height_reference_policy": "source_stamped_ground_plane_v1", "map_identity": self.prior.map_identity, "frame": "map", "revision": self.revision, "navigation_z_band": [self.navigation_z_min, self.navigation_z_max], "observed_cells": _cells_to_json(self.observed), "visibility_cells": _cells_to_json(self.visibility), "live_obstacle_cells": _cells_to_json(self.live_obstacles), "accepted_observation_ids": sorted(self._accepted_ids), "retry_regions": [{"id": region["id"], "cells": _cells_to_json(region["cells"]), "attempts": region["attempts"]} for region in sorted(self._retry_regions, key=lambda item: item["id"])]}

    def import_state(self, state):
        if not isinstance(state, dict) or state.get("state_version") != self.STATE_VERSION or state.get("height_reference_policy") != "source_stamped_ground_plane_v1" or state.get("map_identity") != self.prior.map_identity or state.get("frame") != "map": raise InputError("incompatible persisted state")
        if state.get("navigation_z_band") != [self.navigation_z_min, self.navigation_z_max]: raise InputError("persisted navigation band mismatch")
        revision, accepted = state.get("revision"), state.get("accepted_observation_ids")
        if not isinstance(revision, int) or revision < 0 or not isinstance(accepted, list) or len(set(accepted)) != len(accepted): raise InputError("invalid persisted state")
        groups = state.get("retry_regions", [])
        if not isinstance(groups, list): raise InputError("invalid persisted retry regions")
        self.observed, self.visibility, self.live_obstacles = _cells_from_json(state.get("observed_cells", [])), _cells_from_json(state.get("visibility_cells", [])), _cells_from_json(state.get("live_obstacle_cells", []))
        if not self.observed.issubset(self.visibility) or any(not self.prior.inside(cell) for cell in self.visibility | self.live_obstacles): raise InputError("invalid persisted cells")
        self.revision, self._accepted_ids, self._issued_regions = revision, set(accepted), {}
        self._retry_regions = [{"id": item["id"], "cells": _cells_from_json(item["cells"]), "attempts": item["attempts"]} for item in groups]

    def _prior_connected_area(self, start):
        if self.prior.occupancy(start) != "free": return 0
        seen, queue = {start}, deque([start])
        while queue:
            cell = queue.popleft()
            for adjacent in _neighbors(cell):
                if adjacent not in seen and self.prior.occupancy(adjacent) == "free": seen.add(adjacent); queue.append(adjacent)
        return len(seen)

def run_cli(input_path, output_path):
    payload = json.loads(Path(input_path).read_text()); prior = load_pinned_prior(payload["prior_manifest"])
    core = ExplorationCore(prior, **payload.get("config", {}))
    if "state" in payload: core.import_state(payload["state"])
    started_cpu = time.process_time(); results = [core.apply_observation(record) for record in payload.get("observations", [])]; shortlist = core.candidates(payload["robot_pose"])
    output = {"observation_results": results, "shortlist": shortlist, "state": core.export_state(), "runtime": {"cpu_only": True, "cpu_process_seconds": round(time.process_time() - started_cpu, 6), "estimated_grid_memory_bytes": prior.width * prior.height * 3}}
    Path(output_path).write_text(json.dumps(output, indent=2, sort_keys=True) + "\n"); return output

def main():
    parser = argparse.ArgumentParser(); parser.add_argument("input"); parser.add_argument("output"); args = parser.parse_args(); run_cli(args.input, args.output)
if __name__ == "__main__": main()

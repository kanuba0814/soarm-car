#!/usr/bin/env python3
"""Loopback-only, read-only robot workspace for archived observations."""
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[2]
INSPECTOR = Path(__file__).resolve().parent
PUBLIC_INPUT = ROOT / "artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input"
VLM_RESULTS = ROOT / "artifacts/companion-home-validation/evidence/vlm-crop-context-v1/results/results.json"
OVERLAY_ROOT = VLM_RESULTS.parent / "overlays"
WEB_ROOT = INSPECTOR / "web"

sys.path.insert(0, str(ROOT))
from semantic_memory.core import DEFAULT_POLICY, import_public_archive, query  # noqa: E402

CATEGORIES = {"椅子": "椅子", "餐桌": "餐桌", "不存在": "不存在"}
REPLAYS = {"fresh": 4_621_000_000, "expired": 4_900_000_000}


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def ensure_finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("evidence contains a non-finite number")
    if isinstance(value, list):
        for item in value:
            ensure_finite(item)
    if isinstance(value, dict):
        for item in value.values():
            ensure_finite(item)


def build_assets():
    assets = {"app.css": WEB_ROOT / "app.css", "app.js": WEB_ROOT / "app.js"}
    for observation_id in ("obs_001", "obs_002", "obs_003", "obs_004"):
        assets[f"memory/{observation_id}.png"] = PUBLIC_INPUT / observation_id / "rgb.png"
        for suffix in ("full_annotated", "crop_contact_sheet", "full_plus_crop_panels", "full_annotated_permuted"):
            assets[f"vlm/{observation_id}_{suffix}.png"] = OVERLAY_ROOT / f"{observation_id}_{suffix}.png"
    missing = [name for name, path in assets.items() if not path.is_file()]
    if missing:
        raise RuntimeError(f"explicit public asset missing: {', '.join(missing)}")
    return assets


def public_candidate(candidate):
    evidence = candidate.get("evidence", {})
    rgb = evidence.get("rgb", {})
    observation_id = candidate.get("observation_id")
    return {
        "candidate_id": candidate.get("candidate_id"),
        "observation_id": observation_id,
        "label": candidate.get("label"),
        "detector_confidence": candidate.get("detector_confidence"),
        "geometry": candidate.get("geometry"),
        "evidence": {
            "image_url": f"/public/memory/{observation_id}.png",
            "rgb_sha256": rgb.get("sha256"),
        },
    }


def build_vlm_payload():
    index = read_json(INSPECTOR / "evidence_index.json")
    results = read_json(VLM_RESULTS)
    records = []
    image_names = {
        "full": "full_annotated",
        "crops": "crop_contact_sheet",
        "context": "full_plus_crop_panels",
        "reversed": "full_annotated_permuted",
    }
    output_names = {
        "full": "full_annotated",
        "crops": "crop_contact_sheet",
        "context": "full_plus_crop_context",
        "reversed": "full_annotated_reversed_labels",
    }
    for record in results["records"]:
        output = record["outputs"]
        records.append({
            "observation_id": record["observation_id"],
            "candidate_ids": [candidate["opaque_id"] for candidate in record["candidates"]],
            "images": {tab: f"/public/vlm/{record['observation_id']}_{image}.png" for tab, image in image_names.items()},
            "outputs": {
                tab: {
                    "raw": output[name]["raw"],
                    "strict_format_valid": output[name]["strict_format_valid"],
                    "finite_label_adapter_valid": output[name]["finite_label_adapter_valid"],
                    "adapter_label": output[name]["adapter_label"],
                    "selected_id": output[name]["selected_id"],
                    "latency_seconds": output[name]["latency_seconds"],
                }
                for tab, name in output_names.items()
            },
        })
    payload = {"summary": index["vlm_crop_context"], "runtime": results["runtime"], "records": records}
    ensure_finite(payload)
    return payload


class InspectorHandler(BaseHTTPRequestHandler):
    server_version = "AssistantInspector/1"

    def log_message(self, format, *args):
        sys.stderr.write("[inspector] " + format % args + "\n")

    def send_json(self, status, value):
        ensure_finite(value)
        encoded = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)

    def send_file(self, path, content_type):
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def reject(self, status, message):
        self.send_json(status, {"error": message, "motion_authorized": False})

    def do_HEAD(self):
        parsed = urlparse(self.path)
        if parsed.query:
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.end_headers()
            return
        if parsed.path == "/":
            path, content_type = WEB_ROOT / "index.html", "text/html; charset=utf-8"
        elif parsed.path.startswith("/public/"):
            path = self.server.assets.get(parsed.path.removeprefix("/public/"))
            content_type = mimetypes.guess_type(path.name)[0] if path else None
        else:
            path, content_type = None, None
        if path is None:
            self.send_response(HTTPStatus.NOT_FOUND)
            self.end_headers()
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(path.stat().st_size))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        parameters = parse_qs(parsed.query, keep_blank_values=True)
        if parsed.path == "/":
            if parameters:
                return self.reject(HTTPStatus.BAD_REQUEST, "页面路由不接受查询参数")
            return self.send_file(WEB_ROOT / "index.html", "text/html; charset=utf-8")
        if parsed.path == "/api/overview":
            if parameters:
                return self.reject(HTTPStatus.BAD_REQUEST, "概览接口不接受查询参数")
            return self.send_json(HTTPStatus.OK, self.server.overview)
        if parsed.path == "/api/vlm":
            if parameters:
                return self.reject(HTTPStatus.BAD_REQUEST, "VLM 接口不接受查询参数")
            return self.send_json(HTTPStatus.OK, self.server.vlm)
        if parsed.path == "/api/query":
            if set(parameters) != {"category", "replay"} or any(len(value) != 1 for value in parameters.values()):
                return self.reject(HTTPStatus.BAD_REQUEST, "仅允许 category 与 replay 各一个白名单值")
            category, replay = parameters["category"][0], parameters["replay"][0]
            if category not in CATEGORIES or replay not in REPLAYS:
                return self.reject(HTTPStatus.BAD_REQUEST, "类别或回放时间不在演示白名单中")
            result = query(self.server.store_path, CATEGORIES[category], REPLAYS[replay], DEFAULT_POLICY["source_clock_id"])
            raw_candidates = result["candidates"]
            result["replay"] = replay
            result["replay_time_ns"] = REPLAYS[replay]
            result["candidate_ids"] = [item["candidate_id"] if isinstance(item, dict) else item for item in raw_candidates]
            result["candidates"] = [public_candidate(candidate) for candidate in raw_candidates if isinstance(candidate, dict)]
            return self.send_json(HTTPStatus.OK, result)
        if parsed.path.startswith("/public/"):
            if parameters:
                return self.reject(HTTPStatus.BAD_REQUEST, "公开证据 URL 不接受查询参数")
            name = parsed.path.removeprefix("/public/")
            asset = self.server.assets.get(name)
            if asset is None:
                return self.reject(HTTPStatus.NOT_FOUND, "不在公开证据白名单中")
            return self.send_file(asset, mimetypes.guess_type(asset.name)[0] or "application/octet-stream")
        self.reject(HTTPStatus.NOT_FOUND, "未允许的只读路由")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error("--output must be a new, nonexistent cache directory; refusing overwrite")
    output.mkdir(parents=True)
    store_path = output / "semantic-memory-store.json"
    import_result = import_public_archive(PUBLIC_INPUT, store_path)
    index = read_json(INSPECTOR / "evidence_index.json")
    overview = {
        "offline": True,
        "motion_authorized": False,
        "semantic_memory": {**index["semantic_memory"], "import_result": import_result},
        "navigation_history": index["navigation_history"],
        "sources": {
            "implementation": "docs/navigation-localization-implementation.md",
            "memory_core": "semantic_memory/core.py",
            "memory_cli": "scripts/query_semantic_memory.py",
            "vlm_verifier": "scripts/verify_vlm_crop_context_results.py",
        },
    }
    ensure_finite(overview)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), InspectorHandler)
    server.store_path = store_path
    server.assets = build_assets()
    server.overview = overview
    server.vlm = build_vlm_payload()
    print(f"assistant inspector listening on http://127.0.0.1:{args.port} (cache: {output})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

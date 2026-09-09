#!/usr/bin/env python3
"""Extract only selected public RGB-D members and sanitized A1 metadata."""
import argparse
import hashlib
import json
import tarfile
from pathlib import Path

FILES = {"rgb.png", "depth_m_float32.npy", "depth_invalid_mask.npy", "camera_info.json"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--metadata-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("refusing to overwrite output directory")
    config = json.loads(args.config.read_text())
    observations = config["observations"]
    expected = "f89bec0a7eef59f7aaee016704b2716cb3149c20389dc62eb9863e376b578e69"
    if digest(args.archive) != expected:
        raise SystemExit("archive hash does not match bound evidence")
    args.output_dir.mkdir(parents=True)
    prefix = "instance-survey-e8974b6-retry1/evaluator-package/public/"
    wanted = {f"{prefix}{observation}/{name}" for observation in observations for name in FILES}
    with tarfile.open(args.archive, "r:gz") as archive:
        missing = wanted - set(archive.getnames())
        if missing:
            raise SystemExit(f"archive missing public members: {sorted(missing)}")
        for name in sorted(wanted):
            destination = args.output_dir / Path(name).relative_to(prefix)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with archive.extractfile(name) as source, destination.open("wb") as target:
                target.write(source.read())
    calibration = json.loads((args.metadata_dir / "calibration_selfpose.json").read_text())
    for name in ("detections.json", "gates.json"):
        content = json.loads((args.metadata_dir / name).read_text())
        content["observations"] = {key: value for key, value in content["observations"].items() if key in observations}
        (args.output_dir / name).write_text(json.dumps(content, indent=2) + "\n")
    selected = {key: value for key, value in calibration["observations"].items() if key in observations}
    (args.output_dir / "calibration_selfpose.json").write_text(json.dumps({"scope": calibration["scope"], "observations": selected}, indent=2) + "\n")
    files = {str(path.relative_to(args.output_dir)): digest(path) for path in sorted(args.output_dir.rglob("*")) if path.is_file()}
    manifest = {"scope": "PUBLIC_RGBD_VLM_GRID_DEV_INPUT", "source_archive_sha256": expected, "observations": observations, "allowed_files": sorted(FILES), "files": files}
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"observations": len(observations), "files": len(files), "private_members_extracted": False}))


if __name__ == "__main__":
    main()

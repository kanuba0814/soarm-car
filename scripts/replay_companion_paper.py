#!/usr/bin/env python3
"""Replay one frozen A1 archive without ROS, rendering, inference or robot access."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=300)
    if result.returncode:
        raise RuntimeError(result.stderr[-4000:] or result.stdout[-4000:])
    return result.stdout.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", choices=("101", "103", "107"), default="101")
    args = parser.parse_args()
    if not shutil.which("bwrap"):
        raise SystemExit("bwrap is required; do not replace the isolated public run with unrestricted access")
    archive_path = ROOT / "artifacts/companion-home-validation/evidence/a1-heldout-results-v6" / f"a1-heldout-seed{args.seed}.tar.xz"
    config = ROOT / "config/a1_visualconf_dev_v1.json"
    work = Path(tempfile.mkdtemp(prefix=f"companion-paper-replay-{args.seed}-"))
    data = work / "data"
    with tarfile.open(archive_path) as archive:
        for member in archive.getmembers():
            path = Path(member.name)
            if path.is_absolute() or ".." in path.parts or not (member.isfile() or member.isdir()):
                raise RuntimeError(f"unsafe archive member: {member.name}")
        archive.extractall(data, filter="data")
    receipt = json.loads((data / "receipt.json").read_text())
    if receipt["a1_config_sha256"] != digest(config):
        raise RuntimeError("current A1 configuration differs from the frozen receipt")
    for relative, expected in receipt["outputs"].items():
        path = (data / relative).resolve()
        if not path.is_relative_to(data.resolve()) or digest(path) != expected:
            raise RuntimeError(f"receipt mismatch: {relative}")
    output_parent = work / "public-output"
    output_parent.mkdir()
    python_root = Path(sys.executable).resolve().parents[1]
    sandbox = ["bwrap"]
    for path in dict.fromkeys((Path("/usr"), Path("/lib"), Path("/lib64"), python_root)):
        if path.exists():
            sandbox += ["--ro-bind", str(path), str(path)]
    for source, destination in ((ROOT / "scripts", "/code"), (data / "package/public", "/public"), (data / "public-input", "/input"), (config, "/config.json")):
        sandbox += ["--ro-bind", str(source), destination]
    sandbox += ["--proc", "/proc", "--dev", "/dev", "--bind", str(output_parent), "/method-parent", sys.executable, "/code/run_a1_visualconf_public.py", "--public-dir", "/public", "--public-input-dir", "/input", "--config", "/config.json", "--output-dir", "/method-parent/method"]
    public_log = run(sandbox)
    evaluation = work / "private-evaluation"
    private_log = run([sys.executable, str(ROOT / "scripts/evaluate_a1_visualconf_private.py"), "--private-dir", str(data / "package/private"), "--public-dir", str(data / "package/public"), "--public-input-dir", str(data / "public-input"), "--method", str(output_parent / "method/method.json"), "--config", str(config), "--output-dir", str(evaluation)])
    original = json.loads((data / "private-evaluation/evaluation.json").read_text())
    replayed = json.loads((evaluation / "evaluation.json").read_text())
    for field in ("rows", "metrics"):
        if original[field] != replayed[field]:
            raise RuntimeError(f"replayed {field} differs from the frozen result; retained at {work}")
    report = {"scope": "Offline replay, not new experimental samples", "seed": args.seed, "archive_sha256": digest(archive_path), "receipt_files_verified": len(receipt["outputs"]), "rows_equal": True, "metrics_equal": True, "rows": len(replayed["rows"]), "public_isolation": "bwrap with no private dataset mount", "output_directory": str(work), "public_stdout": public_log, "private_stdout": private_log}
    (work / "replay-verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=4) + "\n")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()

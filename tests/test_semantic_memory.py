import copy
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from semantic_memory.core import DEFAULT_POLICY, ValidationError, _append_revision, import_public_archive, load_store, query


ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input"
CLOCK = DEFAULT_POLICY["source_clock_id"]


class SemanticMemoryTests(unittest.TestCase):
    def _import(self, directory):
        store = Path(directory) / "memory.json"
        result = import_public_archive(PUBLIC, store)
        return store, result

    def test_actual_archive_import_is_idempotent_and_observation_local(self):
        with tempfile.TemporaryDirectory() as directory:
            store, first = self._import(directory)
            second = import_public_archive(PUBLIC, store)
            loaded = load_store(store)
            self.assertEqual(first["records"], 11)
            self.assertEqual(first["imported_revisions"], 11)
            self.assertEqual(second["imported_revisions"], 0)
            self.assertEqual(len(loaded["records"]), 11)
            self.assertEqual(sorted(loaded["records"]), [
                "obs_001:0", "obs_001:1", "obs_002:0", "obs_002:1", "obs_002:2",
                "obs_002:3", "obs_003:0", "obs_003:1", "obs_003:2", "obs_004:0", "obs_004:2",
            ])
            for bucket in loaded["records"].values():
                record = bucket["revisions"][0]["record"]
                self.assertIn(record["geometry"]["status"], {"ok", "rejected"})
                self.assertTrue((PUBLIC / record["evidence"]["rgb"]["path"]).is_file())
                self.assertEqual(record["evidence"]["rgb"]["sha256"], hashlib.sha256((PUBLIC / record["evidence"]["rgb"]["path"]).read_bytes()).hexdigest())

    def test_fresh_process_cli_returns_expired_refresh_and_alias_results(self):
        with tempfile.TemporaryDirectory() as directory:
            store, _ = self._import(directory)
            command = [sys.executable, "scripts/query_semantic_memory.py", "--store", str(store)]
            def run(category, stamp, *extra):
                completed = subprocess.run(command + ["--category", category, "--replay-time-ns", str(stamp), *extra], cwd=ROOT, check=True, capture_output=True, text=True)
                return json.loads(completed.stdout)
            fresh_table = run("餐桌", 4_600_000_000)
            self.assertEqual(fresh_table["status"], "candidates")
            self.assertFalse(fresh_table["motion_authorized"])
            self.assertEqual(run("chair", 4_621_000_000)["status"], "needs_confirmation")
            self.assertEqual(run("diningtable", 4_900_000_000)["status"], "needs_confirmation")
            self.assertEqual(run("lamp", 4_600_000_000)["status"], "not_found")
            self.assertEqual(run("餐桌", 4_600_000_000, "--source-clock-id", "unknown")["reason"], "incompatible_clock")
            self.assertEqual(run("餐桌", 3_000_000_000)["reason"], "future_source_stamp")

    def test_out_of_order_revision_never_becomes_current(self):
        with tempfile.TemporaryDirectory() as directory:
            store, _ = self._import(directory)
            data = load_store(store)
            bucket = data["records"]["obs_003:1"]
            newer = copy.deepcopy(bucket["revisions"][0]["record"])
            newer["source_stamp_ns"] = 9_000_000_000
            newer["geometry"]["depth_median_m"] += 0.01
            older = copy.deepcopy(newer)
            older["source_stamp_ns"] = 8_000_000_000
            self.assertTrue(_append_revision(data, newer))
            self.assertTrue(_append_revision(data, older))
            store.write_text(json.dumps(data), encoding="utf-8")
            result = query(store, "diningtable", 9_100_000_000, CLOCK)
            self.assertEqual(result["status"], "candidates")
            self.assertEqual(result["candidates"][0]["source_stamp_ns"], 9_000_000_000)

    def test_conflicting_revisions_are_retained_and_not_cleared_by_newer_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            store, _ = self._import(directory)
            data = load_store(store)
            bucket = data["records"]["obs_003:1"]
            original = bucket["revisions"][0]["record"]
            conflict = copy.deepcopy(original)
            conflict["geometry"]["depth_median_m"] += 0.1
            newer = copy.deepcopy(original)
            newer["source_stamp_ns"] += 1_000_000_000
            self.assertTrue(_append_revision(data, conflict))
            self.assertTrue(_append_revision(data, newer))
            store.write_text(json.dumps(data), encoding="utf-8")
            result = query(store, "diningtable", newer["source_stamp_ns"] + 1, CLOCK)
            self.assertEqual(result["status"], "needs_confirmation")
            self.assertEqual(result["reason"], "conflicting_revisions")
            self.assertEqual(result["revision_counts"]["obs_003:1"], 3)

    def test_rejects_hash_mismatch_and_nonfinite_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            copied = Path(directory) / "public"
            shutil.copytree(PUBLIC, copied)
            manifest = json.loads((copied / "manifest.json").read_text())
            manifest["files"]["detections.json"] = "0" * 64
            (copied / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValidationError, "hash mismatch"):
                import_public_archive(copied, Path(directory) / "bad-hash.json")
            failed_hash_cli = subprocess.run(
                [sys.executable, "scripts/import_semantic_memory.py", "--public-dir", str(copied), "--store", str(Path(directory) / "bad-hash-cli.json")],
                cwd=ROOT, capture_output=True, text=True,
            )
            self.assertNotEqual(failed_hash_cli.returncode, 0)
            self.assertIn("hash mismatch", failed_hash_cli.stderr)
            shutil.rmtree(copied)
            shutil.copytree(PUBLIC, copied)
            calibration_path = copied / "calibration_selfpose.json"
            calibration = json.loads(calibration_path.read_text())
            calibration["observations"]["obs_001"]["self_pose"]["entity"]["position"]["x"] = float("nan")
            calibration_path.write_text(json.dumps(calibration))
            manifest = json.loads((copied / "manifest.json").read_text())
            manifest["files"]["calibration_selfpose.json"] = hashlib.sha256(calibration_path.read_bytes()).hexdigest()
            (copied / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValidationError, "nonfinite calibration"):
                import_public_archive(copied, Path(directory) / "bad-calibration.json")
            failed_calibration_cli = subprocess.run(
                [sys.executable, "scripts/import_semantic_memory.py", "--public-dir", str(copied), "--store", str(Path(directory) / "bad-calibration-cli.json")],
                cwd=ROOT, capture_output=True, text=True,
            )
            self.assertNotEqual(failed_calibration_cli.returncode, 0)
            self.assertIn("nonfinite calibration", failed_calibration_cli.stderr)


if __name__ == "__main__":
    unittest.main()

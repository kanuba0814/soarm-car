# CPU Semantic Memory

This auxiliary retrieval capability imports the archived public RGB-D observations into a local JSON store. It does not import ROS, Torch, or Transformers, and it never produces a navigation goal or a motion command. Every query response contains `motion_authorized: false`.

## Policy and provenance

The importer verifies every one of the 19 SHA-256 entries in the public manifest before reading the archive. It preserves the opaque observation ID, detection index, source stamp, source-clock identity, RGB/depth/mask/camera-info/calibration paths and digests. A detection becomes one observation-local candidate; category equality never associates, merges, or deduplicates candidates. The stored `calibration_self_pose_xy_m` is only a calibration/self-pose calculation for evidence review. It has no navmap alignment or navigation meaning.

The explicit policy is schema version 1: source clock `public-vlm-grid-dev-v1-source-stamp-ns`, TTL `1500000000` ns, and aliases `chair`/`椅子` plus `diningtable`/`dining table`/`餐桌`. Freshness compares controlled `--replay-time-ns` only against a record with the same source clock. It never compares wall clock time to simulation/source stamps. Unknown or incompatible clocks, future source stamps, expiry, multiple candidates, and open revision conflicts return `needs_confirmation`.

A record is updated atomically through a temporary file and `os.replace`. Exact repeated imports are idempotent. Older revisions are retained but cannot become current; conflicting same-current-stamp revisions are retained and leave `conflict_open` set, including after later evidence. Test-only injected revisions model conflicting records, not observed object movement.

## Reproduction

Run these commands from the repository root:

```bash
python3 scripts/import_semantic_memory.py \
  --public-dir artifacts/companion-home-validation/evidence/vlm-grid-dev-v1/public-input \
  --store artifacts/semantic-memory-validation/public-store.json
python3 scripts/query_semantic_memory.py \
  --store artifacts/semantic-memory-validation/public-store.json \
  --category 餐桌 --replay-time-ns 4600000000
python3 scripts/query_semantic_memory.py \
  --store artifacts/semantic-memory-validation/public-store.json \
  --category chair --replay-time-ns 4621000000
python3 scripts/query_semantic_memory.py \
  --store artifacts/semantic-memory-validation/public-store.json \
  --category diningtable --replay-time-ns 4900000000
python3 -m unittest tests/test_semantic_memory.py -v
```

The checked compact outputs are `artifacts/semantic-memory-validation/results.json` and `artifacts/semantic-memory-validation/policy-tests.json`. They report 11 imported observation-local candidates, a fresh single dining-table result, separate-chair ambiguity, and expiration confirmation. Full generated stores and images are intentionally not committed.

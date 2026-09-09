#!/usr/bin/env python3
"""Query CPU semantic memory using an explicit replay clock."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_memory.core import DEFAULT_POLICY, query


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--store", type=Path, required=True)
    parser.add_argument("--category", required=True)
    parser.add_argument("--replay-time-ns", type=int, required=True)
    parser.add_argument("--source-clock-id", default=DEFAULT_POLICY["source_clock_id"])
    args = parser.parse_args()
    print(json.dumps(query(args.store, args.category, args.replay_time_ns, args.source_clock_id), sort_keys=True))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Import a verified public RGB-D archive into CPU semantic memory."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from semantic_memory.core import import_public_archive


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-dir", type=Path, required=True)
    parser.add_argument("--store", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(import_public_archive(args.public_dir, args.store), sort_keys=True))


if __name__ == "__main__":
    main()

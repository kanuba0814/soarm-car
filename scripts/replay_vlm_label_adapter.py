#!/usr/bin/env python3
"""DEV-only finite-label adapter; never infers identity or authorizes motion."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def adapt_label(raw, labels):
    # Allow exactly one terminal full stop, not arbitrary prose or fuzzy matching.
    match = re.fullmatch(r"([0-9]+|NONE)\.?", raw.strip())
    if not match:
        return None
    label = match.group(1)
    return label if label == "NONE" or label in labels else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    rows = []
    for record in data["records"]:
        mapping = record["candidate_label_to_opaque_id"]
        raw = record["raw_outputs"]["simple_id"]
        label = adapt_label(raw, mapping)
        rows.append({
            "observation_id": record["observation_id"], "raw": raw,
            "adapted_label": label, "opaque_id": mapping.get(label),
            "format_accepted": label is not None,
            "semantic_correctness": "NOT_EVALUATED", "motion_authorized": False,
        })
    result = {
        "adapter": "finite-label-optional-period-v1",
        "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "records": rows, "format_accepted": sum(row["format_accepted"] for row in rows),
        "total": len(rows), "scope": "saved DEV response replay; not new inference or accuracy",
    }
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=4, ensure_ascii=False)
        stream.write("\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Generate paper tables from immutable A1 held-out archive evaluations."""
import argparse
import csv
import hashlib
import json
import subprocess
import tarfile
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVES = ROOT / "artifacts/companion-home-validation/evidence/a1-heldout-results-v6"
VARIANTS = ("V", "V+R", "V+M", "V+R+M")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive_json(path, member):
    with tarfile.open(path, "r:xz") as archive:
        stream = archive.extractfile(member)
        if stream is None:
            raise RuntimeError(f"{path}: missing {member}")
        return json.load(stream)


def fraction(numerator, denominator):
    return None if denominator == 0 else numerator / denominator


def aggregate(rows):
    total = len(rows)
    definite = [row for row in rows if row["gt_status"] == "definite"]
    known_mrr = [row["mrr"] for row in definite if row["mrr"] is not None]
    lower = [row["mrr_lower_bound"] for row in definite if row["mrr_lower_bound"] is not None]
    upper = [row["mrr_upper_bound"] for row in definite if row["mrr_upper_bound"] is not None]
    accepted = [row for row in definite if row["accepted_track_id"] is not None]
    accepted_known = [row for row in accepted if row["accepted_correct"] is not None]
    wrong_known = sum(row["accepted_correct"] is False for row in accepted_known)
    uncertain_accepted = len(accepted) - len(accepted_known)
    top1_known = sum(row["forced_top1"] is not None for row in definite)
    return {
        "total_rows": total, "definite_gt_rows": len(definite), "unknown_gt_rows": total - len(definite),
        "forced_top1_known": top1_known,
        "forced_top1_correct": sum(row["forced_top1"] is True for row in definite),
        "forced_top1_rate_known": fraction(sum(row["forced_top1"] is True for row in definite), top1_known),
        "mrr_exact_known_count": len(known_mrr), "mrr_exact_known": fraction(sum(known_mrr), len(known_mrr)),
        "mrr_lower_bound": fraction(sum(lower), len(lower)), "mrr_upper_bound": fraction(sum(upper), len(upper)),
        "accepted_total": len(accepted), "accepted_known": len(accepted_known),
        "accepted_correct_known": sum(row["accepted_correct"] is True for row in accepted_known),
        "accepted_unknown": uncertain_accepted, "accepted_coverage_definite": fraction(len(accepted), len(definite)),
        "wrong_accepted_known": wrong_known,
        "wrong_accepted_rate_lower": fraction(wrong_known, len(accepted)),
        "wrong_accepted_rate_upper": fraction(wrong_known + uncertain_accepted, len(accepted)),
        "relation_unknown_rows": sum(row["public_relation_status"] == "unknown" for row in rows),
        "relation_unknown_rate": fraction(sum(row["public_relation_status"] == "unknown" for row in rows), total),
    }


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save(fig, output, name):
    for suffix in ("svg", "png"):
        fig.savefig(output / f"{name}.{suffix}", bbox_inches="tight", dpi=180)
    plt.close(fig)


def figures(output, by_variant, by_layout):
    labels = [item["variant"] for item in by_variant]
    fig, axis = plt.subplots(figsize=(8, 3.8)); positions = list(range(len(labels)))
    axis.bar(positions, [item["forced_top1_rate_known"] or 0 for item in by_variant], label="forced Top-1 (known)", color="#277da1")
    axis.bar(positions, [item["accepted_coverage_definite"] or 0 for item in by_variant], bottom=[item["forced_top1_rate_known"] or 0 for item in by_variant], label="accepted coverage (display stack)", color="#90be6d")
    axis.set_xticks(positions, labels); axis.set_ylim(0, 1.35); axis.set_ylabel("rate")
    axis.set_title("A1 held-out outcomes by variant; unknown reported separately")
    for index, item in enumerate(by_variant):
        axis.text(index, 1.05, f"unknown GT {item['unknown_gt_rows']}/{item['total_rows']}\naccepted unknown {item['accepted_unknown']}/{item['accepted_total']}", ha="center", fontsize=8)
    axis.legend(fontsize=8, loc="upper right"); save(fig, output, "pervariant-outcomes")
    fig, axis = plt.subplots(figsize=(8, 3.8)); labels = [item["scene_id"].replace("a1-heldout-", "") for item in by_layout]
    positions = list(range(len(labels)))
    axis.bar(positions, [item["relation_unknown_rate"] or 0 for item in by_layout], color="#f8961e", label="relation unknown / all rows")
    axis.bar(positions, [item["unknown_gt_rows"] / item["total_rows"] for item in by_layout], color="#f94144", alpha=.7, label="unknown GT / all rows")
    axis.set_xticks(positions, labels); axis.set_ylim(0, 1); axis.set_ylabel("fraction")
    axis.set_title("A1 held-out coverage and failure conditions by layout"); axis.legend(fontsize=8)
    save(fig, output, "perlayout-failure-coverage")
    fig, axis = plt.subplots(figsize=(8, 2.8)); axis.axis("off")
    steps = ["immutable .tar.xz\npublic RGB-D + inputs", "bwrap public method\nV / V+R / V+M / V+R+M", "private evaluator\nGT masks + identities", "raw-row aggregation\nCSV / JSON / figures"]
    for index, text in enumerate(steps):
        x = .03 + index * .245
        axis.text(x, .5, text, ha="center", va="center", transform=axis.transAxes, bbox={"boxstyle": "round,pad=.4", "fc": "#f1faee", "ec": "#457b9d"})
        if index < len(steps) - 1:
            axis.annotate("", xy=(x + .19, .5), xytext=(x + .12, .5), xycoords=axis.transAxes, arrowprops={"arrowstyle": "->"})
    axis.set_title("Frozen A1 evidence and isolation pipeline"); save(fig, output, "pipeline")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archives", type=Path, default=DEFAULT_ARCHIVES)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/paper/results")
    args = parser.parse_args()
    archives = sorted(args.archives.glob("a1-heldout-seed*.tar.xz"))
    if len(archives) != 3:
        raise SystemExit(f"expected exactly three archives, found {len(archives)}")
    args.output.mkdir(parents=True, exist_ok=True)
    all_rows, receipt_trace = [], []
    for archive in archives:
        evaluation = archive_json(archive, "private-evaluation/evaluation.json")
        receipt = archive_json(archive, "receipt.json")
        scene_id, rows = receipt["scene_id"], evaluation["rows"]
        if len(rows) != 24 or {row["variant"] for row in rows} != set(VARIANTS):
            raise RuntimeError(f"{archive}: expected 24 rows and four frozen variants")
        all_rows.extend({"scene_id": scene_id, **row} for row in rows)
        receipt_trace.append({"archive": archive.name, "archive_sha256": sha256(archive), "scene_id": scene_id, "evaluation_member": "private-evaluation/evaluation.json", "evaluation_sha256": receipt["outputs"]["private-evaluation/evaluation.json"], "method_member": "public-output/method/method.json", "method_sha256": receipt["outputs"]["public-output/method/method.json"]})
    if len(all_rows) != 72 or len({(r["scene_id"], r["variant"], r["query_id"]) for r in all_rows}) != 72:
        raise RuntimeError("raw record uniqueness/count invariant failed")
    by_variant = [{"variant": variant, **aggregate([row for row in all_rows if row["variant"] == variant])} for variant in VARIANTS]
    scenes = sorted({row["scene_id"] for row in all_rows})
    by_layout = [{"scene_id": scene, **aggregate([row for row in all_rows if row["scene_id"] == scene])} for scene in scenes]
    by_layout_variant = [{"scene_id": scene, "variant": variant, **aggregate([row for row in all_rows if row["scene_id"] == scene and row["variant"] == variant])} for scene in scenes for variant in VARIANTS]
    write_csv(args.output / "a1-heldout-raw-rows.csv", all_rows); write_csv(args.output / "a1-heldout-by-variant.csv", by_variant)
    write_csv(args.output / "a1-heldout-by-layout.csv", by_layout); write_csv(args.output / "a1-heldout-by-layout-variant.csv", by_layout_variant)
    report = {"scope": "three held-out layouts of one AWS Small House; no pooling as independent homes", "code_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(), "archives": receipt_trace, "counts": {"rows": len(all_rows), "variants": len(VARIANTS), "layouts": len(scenes), "views": 24, "query_layout_conditions": 18}, "definitions": {"exact_mrr": "mean only over definite rows whose mrr field is non-null", "mrr_bounds": "means of private-evaluator per-row lower/upper MRR bounds on definite rows", "wrong_accepted_bounds": "unknown accepted identities are neither correct nor wrong; lower=known wrong/accepted total and upper=(known wrong+unknown accepted)/accepted total"}, "by_variant": by_variant, "by_layout": by_layout, "by_layout_variant": by_layout_variant, "traceability": "Each numeric table value derives from private-evaluation/evaluation.json.rows and the archive SHA-256 listed above."}
    (args.output / "a1-heldout-results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    figures(args.output, by_variant, by_layout)
    print(json.dumps({"rows": len(all_rows), "views": 24, "query_layout_conditions": 18, "variants": len(VARIANTS), "layouts": len(scenes), "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()

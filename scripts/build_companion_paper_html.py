#!/usr/bin/env python3
"""Bounded offline HTML renderer for the editable A1 paper Markdown."""
import argparse
import html
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def render(markdown):
    blocks = []
    paragraph = []
    def flush():
        if paragraph:
            blocks.append("<p>" + html.escape(" ".join(paragraph)) + "</p>")
            paragraph.clear()
    for line in markdown.splitlines():
        if line.startswith("#"):
            flush(); level = len(line) - len(line.lstrip("#")); blocks.append(f"<h{level}>{html.escape(line[level:].strip())}</h{level}>")
        elif line.startswith("!"):
            flush(); match = re.match(r"!\[[^]]*\]\(([^)]+)\)", line)
            if match: blocks.append(f'<img src="{html.escape(match.group(1), quote=True)}" alt="figure">')
        elif not line.strip(): flush()
        elif line.startswith("|"):
            flush(); blocks.append("<pre>" + html.escape(line) + "</pre>")
        else: paragraph.append(line)
    flush()
    return "\n".join(blocks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "docs/paper/thesis.md")
    parser.add_argument("--output", type=Path, default=ROOT / "docs/paper/paper.html")
    args = parser.parse_args()
    body = render(args.input.read_text(encoding="utf-8"))
    page = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>A1 留出评估论文</title><style>body{{max-width:900px;margin:32px auto;padding:0 20px;font-family:"Noto Sans CJK SC","Noto Sans SC",sans-serif;line-height:1.75;color:#182126}}h1{{font-size:2em}}h2{{margin-top:2.1em;border-bottom:1px solid #b9c7ce}}h3{{margin-top:1.5em}}p{{text-align:justify}}pre{{white-space:pre-wrap;background:#f4f7f8;padding:10px;border-left:3px solid #277da1}}img{{max-width:100%;height:auto;margin:18px 0;border:1px solid #ccd6da}}code{{font-family:monospace}}</style></head><body>{body}<h2>生成图</h2><img src="results/pipeline.svg" alt="pipeline"><img src="results/pervariant-outcomes.svg" alt="per-variant outcomes"><img src="results/perlayout-failure-coverage.svg" alt="per-layout failure and coverage"></body></html>'''
    args.output.write_text(page, encoding="utf-8")
    print(args.output)

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Build an offline HTML edition of the A1 manuscript with local citations."""
import argparse
import html
import re
from pathlib import Path

from markdown_it import MarkdownIt

ROOT = Path(__file__).resolve().parents[1]
CITATION = re.compile(r"\\cite\{([^}]+)\}")
BIB_ENTRY = re.compile(r"@\w+\{([^,]+),\s*(.*?)\}\s*(?=@|\Z)", re.S)
FIELD = re.compile(r"(title|year|eprint|archivePrefix|url)\s*=\s*\{([^}]*)\}", re.I)


def bibliography(path):
    entries = {}
    for key, content in BIB_ENTRY.findall(path.read_text(encoding="utf-8")):
        entries[key] = {name.lower(): value for name, value in FIELD.findall(content)}
    return entries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "docs/paper/thesis.md")
    parser.add_argument("--bib", type=Path, default=ROOT / "docs/paper/references.bib")
    parser.add_argument("--output", type=Path, default=ROOT / "docs/paper/paper.html")
    args = parser.parse_args()
    refs, cited = bibliography(args.bib), []

    def citation(match):
        keys = match.group(1).split(",")
        numbers = []
        for key in keys:
            key = key.strip()
            if key not in refs:
                raise SystemExit(f"missing bibliography key: {key}")
            if key not in cited:
                cited.append(key)
            numbers.append(str(cited.index(key) + 1))
        return "[" + ", ".join(numbers) + "]"

    markdown = CITATION.sub(citation, args.input.read_text(encoding="utf-8"))
    figures = "\n\n## 图\n\n![图 1：冻结 A1 公共/私有隔离与聚合链](results/pipeline.svg)\n\n图 1：归档只读，公共方法和私有 evaluator 分离。\n\n![图 2：按变体的 forced/accepted 结果与 unknown 计数](results/pervariant-outcomes.svg)\n\n图 2：unknown 明确标注，柱形不表示合并成功率。\n\n![图 3：按布局的 relation-unknown 与 unknown GT 覆盖](results/perlayout-failure-coverage.svg)\n\n图 3：三个 layout 属于同一住宅。\n"
    reference_lines = ["## 参考文献"]
    for index, key in enumerate(cited, 1):
        entry = refs[key]
        parts = [entry.get("title", key), entry.get("year", "")]
        if entry.get("eprint"):
            parts.append("arXiv:" + entry["eprint"])
        if entry.get("url"):
            parts.append(entry["url"])
        reference_lines.append(f"{index}. " + ". ".join(part for part in parts if part))
    markdown = re.sub(r"## 参考文献[\s\S]*$", "", markdown).rstrip() + figures + "\n\n" + "\n".join(reference_lines) + "\n"
    body = MarkdownIt("commonmark", {"html": True}).enable("table").render(markdown)
    if "\\cite" in body or "**" in body:
        raise SystemExit("unrendered citation or emphasis marker")
    page = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>面向家庭服务机器人的关系与多视角视觉目标辨识</title><style>body{{max-width:900px;margin:32px auto;padding:0 20px;font-family:"Noto Sans CJK SC","Noto Sans SC",sans-serif;line-height:1.75;color:#182126}}h1{{font-size:2em}}h2{{margin-top:2.1em;border-bottom:1px solid #b9c7ce}}h3{{margin-top:1.5em}}p{{text-align:justify}}table{{border-collapse:collapse;width:100%;font-size:.9em}}td,th{{border:1px solid #b9c7ce;padding:6px;vertical-align:top}}img{{max-width:100%;height:auto;margin:18px 0 4px;border:1px solid #ccd6da}}code{{font-family:monospace;background:#f4f7f8;padding:1px 3px}}blockquote{{border-left:3px solid #277da1;margin-left:0;padding-left:12px}}@media print{{body{{max-width:none;margin:0;padding:0;font-size:10pt}}h1,h2,h3{{break-after:avoid-page}}p{{orphans:3;widows:3}}img,table{{break-inside:avoid}}a{{overflow-wrap:anywhere}}}}@page{{margin:18mm}}</style></head><body>{body}</body></html>'''
    args.output.write_text(page, encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()

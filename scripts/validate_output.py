#!/usr/bin/env python3
"""Validation script for the generated Nav2 documentation markdown."""

import argparse
import re
import sys
from pathlib import Path


def validate_markdown(filepath: str) -> dict:
    """
    Validate the generated markdown file.

    Returns a dict with validation results.
    """
    results = {
        "valid": True,
        "errors": [],
        "warnings": [],
        "stats": {},
    }

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            content = f.read()
    except FileNotFoundError:
        results["valid"] = False
        results["errors"].append(f"File not found: {filepath}")
        return results
    except Exception as e:
        results["valid"] = False
        results["errors"].append(f"Error reading file: {e}")
        return results

    lines = content.split("\n")

    # Basic stats
    results["stats"]["file_size_kb"] = len(content.encode("utf-8")) / 1024
    results["stats"]["line_count"] = len(lines)
    results["stats"]["word_count"] = len(content.split())

    # Count headings
    headings = re.findall(r"^(#{1,6})\s+(.+)$", content, re.MULTILINE)
    results["stats"]["heading_count"] = len(headings)

    heading_levels = {}
    for hashes, text in headings:
        level = len(hashes)
        heading_levels[level] = heading_levels.get(level, 0) + 1
    results["stats"]["heading_levels"] = heading_levels

    # Count code blocks
    code_blocks = re.findall(r"```[\s\S]*?```", content)
    results["stats"]["code_block_count"] = len(code_blocks)

    # Count links
    links = re.findall(r"\[([^\]]+)\]\(([^)]+)\)", content)
    results["stats"]["link_count"] = len(links)

    # Count tables
    table_rows = re.findall(r"^\|.+\|$", content, re.MULTILINE)
    results["stats"]["table_row_count"] = len(table_rows)

    # Validation checks

    # 1. Check for empty file
    if len(content.strip()) == 0:
        results["valid"] = False
        results["errors"].append("File is empty")
        return results

    # 2. Check for title
    if not content.strip().startswith("#"):
        results["warnings"].append("File does not start with a heading")

    # 3. Check for broken internal links (anchors)
    anchor_pattern = re.compile(r"\]\(#([^)]+)\)")
    anchors_referenced = anchor_pattern.findall(content)

    # Extract available anchors from headings
    available_anchors = set()
    for hashes, text in headings:
        # Create anchor from heading text (simplified)
        anchor = text.lower()
        anchor = re.sub(r"[^a-z0-9\s-]", "", anchor)
        anchor = re.sub(r"\s+", "-", anchor)
        anchor = re.sub(r"-+", "-", anchor).strip("-")
        available_anchors.add(anchor)

    broken_anchors = []
    for anchor in anchors_referenced:
        normalized = anchor.lower().strip()
        if normalized not in available_anchors:
            broken_anchors.append(anchor)

    if broken_anchors:
        results["warnings"].append(
            f"Potentially broken internal links: {len(broken_anchors)} "
            f"(first 5: {broken_anchors[:5]})"
        )

    # 4. Check for unclosed code blocks
    backtick_count = content.count("```")
    if backtick_count % 2 != 0:
        results["errors"].append("Unclosed code block detected (odd number of ```)")
        results["valid"] = False

    # 5. Check heading hierarchy
    last_level = 0
    for hashes, text in headings:
        level = len(hashes)
        if level > last_level + 1 and last_level > 0:
            results["warnings"].append(
                f"Heading level skip: jumped from h{last_level} to h{level} at '{text[:50]}...'"
            )
            break  # Only report first occurrence
        last_level = level

    # 6. Check for very long lines (may cause rendering issues)
    long_lines = sum(1 for line in lines if len(line) > 500)
    if long_lines > 0:
        results["warnings"].append(f"{long_lines} lines exceed 500 characters")

    # 7. Check for minimum content
    if results["stats"]["word_count"] < 100:
        results["warnings"].append("Document has very little content (< 100 words)")

    # 8. Check file size
    if results["stats"]["file_size_kb"] > 50000:  # 50MB
        results["warnings"].append("File size exceeds 50MB")

    return results


def print_results(results: dict) -> None:
    """Print validation results in a formatted way."""
    print("\n" + "=" * 60)
    print("VALIDATION RESULTS")
    print("=" * 60)

    # Stats
    print("\nDocument Statistics:")
    stats = results["stats"]
    print(f"  - File size: {stats.get('file_size_kb', 0):.1f} KB")
    print(f"  - Lines: {stats.get('line_count', 0):,}")
    print(f"  - Words: {stats.get('word_count', 0):,}")
    print(f"  - Headings: {stats.get('heading_count', 0)}")
    print(f"  - Code blocks: {stats.get('code_block_count', 0)}")
    print(f"  - Links: {stats.get('link_count', 0)}")
    print(f"  - Table rows: {stats.get('table_row_count', 0)}")

    if "heading_levels" in stats:
        print("  - Heading distribution:")
        for level, count in sorted(stats["heading_levels"].items()):
            print(f"      h{level}: {count}")

    # Errors
    if results["errors"]:
        print(f"\n❌ ERRORS ({len(results['errors'])}):")
        for error in results["errors"]:
            print(f"  - {error}")

    # Warnings
    if results["warnings"]:
        print(f"\n⚠️  WARNINGS ({len(results['warnings'])}):")
        for warning in results["warnings"]:
            print(f"  - {warning}")

    # Overall status
    print("\n" + "-" * 60)
    if results["valid"]:
        if results["warnings"]:
            print("✅ VALID (with warnings)")
        else:
            print("✅ VALID")
    else:
        print("❌ INVALID")
    print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Validate generated Nav2 documentation markdown"
    )
    parser.add_argument(
        "file",
        nargs="?",
        default="data/output/nav2_documentation_complete.md",
        help="Path to the markdown file to validate",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output results as JSON",
    )

    args = parser.parse_args()

    results = validate_markdown(args.file)

    if args.json:
        import json
        print(json.dumps(results, indent=2))
    else:
        print_results(results)

    return 0 if results["valid"] else 1


if __name__ == "__main__":
    sys.exit(main())

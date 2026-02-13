"""Markdown generator module for converting HTML to clean markdown."""

import logging
import re
from typing import List

from markdownify import markdownify as md

from .models import Page, Parameter

logger = logging.getLogger(__name__)


class MarkdownGenerator:
    """Converts parsed HTML content to clean markdown format."""

    def __init__(
        self,
        code_fence_style: str = "```",
        heading_style: str = "atx",
    ):
        self.code_fence_style = code_fence_style
        self.heading_style = heading_style

    def html_to_markdown(self, html: str) -> str:
        """Convert HTML to markdown using markdownify."""
        markdown = md(
            html,
            heading_style=self.heading_style,
            code_language="",
            strip=["script", "style"],
        )

        # Clean up the output
        markdown = self.sanitize_markdown(markdown)

        return markdown

    def sanitize_markdown(self, markdown: str) -> str:
        """Clean up and normalize markdown content."""
        # Remove excessive blank lines (more than 2)
        markdown = re.sub(r"\n{4,}", "\n\n\n", markdown)

        # Remove trailing whitespace from lines
        markdown = "\n".join(line.rstrip() for line in markdown.split("\n"))

        # Ensure code blocks have proper formatting
        markdown = self._fix_code_blocks(markdown)

        # Fix broken links
        markdown = self._fix_links(markdown)

        # Remove empty headers
        markdown = re.sub(r"^#+\s*$", "", markdown, flags=re.MULTILINE)

        return markdown.strip()

    def _fix_code_blocks(self, markdown: str) -> str:
        """Ensure code blocks are properly formatted."""
        # Convert indented code blocks to fenced
        lines = markdown.split("\n")
        result = []
        in_code_block = False
        code_buffer = []

        for line in lines:
            # Check for fenced code blocks
            if line.strip().startswith("```"):
                if in_code_block:
                    in_code_block = False
                else:
                    in_code_block = True
                result.append(line)
            elif in_code_block:
                result.append(line)
            else:
                # Check for indented code (4 spaces or 1 tab)
                if line.startswith("    ") or line.startswith("\t"):
                    code_buffer.append(line[4:] if line.startswith("    ") else line[1:])
                else:
                    # Flush code buffer if we have code
                    if code_buffer:
                        result.append(f"{self.code_fence_style}")
                        result.extend(code_buffer)
                        result.append(f"{self.code_fence_style}")
                        code_buffer = []
                    result.append(line)

        # Flush remaining code buffer
        if code_buffer:
            result.append(f"{self.code_fence_style}")
            result.extend(code_buffer)
            result.append(f"{self.code_fence_style}")

        return "\n".join(result)

    def _fix_links(self, markdown: str) -> str:
        """Fix and clean up markdown links."""
        # Remove empty links
        markdown = re.sub(r"\[([^\]]*)\]\(\s*\)", r"\1", markdown)

        # Fix double-encoded URLs
        markdown = re.sub(r"%25", "%", markdown)

        return markdown

    def fix_heading_levels(self, markdown: str, base_level: int) -> str:
        """Adjust heading levels to fit within document hierarchy."""
        def adjust_heading(match):
            hashes = match.group(1)
            text = match.group(2)
            new_level = min(len(hashes) + base_level - 1, 6)
            return "#" * new_level + " " + text

        # Match ATX-style headings
        pattern = r"^(#{1,6})\s+(.+)$"
        adjusted = re.sub(pattern, adjust_heading, markdown, flags=re.MULTILINE)

        return adjusted

    def format_parameters(self, parameters: List[Parameter]) -> str:
        """Format parameters as markdown."""
        if not parameters:
            return ""

        lines = ["", "#### Parameters", ""]

        for param in parameters:
            lines.append(f"**`{param.name}`**")
            lines.append(f"- Type: `{param.type}`")
            if param.default:
                lines.append(f"- Default: `{param.default}`")
            lines.append(f"- {param.description}")
            lines.append("")

        return "\n".join(lines)

    def page_to_markdown(self, page: Page, heading_level: int = 2) -> str:
        """Convert a Page object to markdown."""
        parts = []

        # Add title
        heading_prefix = "#" * heading_level
        parts.append(f"{heading_prefix} {page.title}")
        parts.append("")

        # Add source URL as reference
        parts.append(f"*Source: [{page.url}]({page.url})*")
        parts.append("")

        # Convert content
        if page.content:
            content_md = self.html_to_markdown(page.content)
            # Adjust heading levels
            content_md = self.fix_heading_levels(content_md, heading_level + 1)
            parts.append(content_md)

        # Add formatted parameters if any
        if page.parameters:
            params_md = self.format_parameters(page.parameters)
            parts.append(params_md)

        return "\n".join(parts)

    def create_anchor(self, text: str) -> str:
        """Create a markdown anchor from text."""
        # Convert to lowercase
        anchor = text.lower()

        # Replace spaces with hyphens
        anchor = re.sub(r"\s+", "-", anchor)

        # Remove non-alphanumeric characters except hyphens
        anchor = re.sub(r"[^a-z0-9-]", "", anchor)

        # Remove multiple consecutive hyphens
        anchor = re.sub(r"-+", "-", anchor)

        return anchor.strip("-")

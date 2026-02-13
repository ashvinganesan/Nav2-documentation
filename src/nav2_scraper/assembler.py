"""Document assembler for creating a concise Nav2 parameter reference."""

import logging
from datetime import datetime
from typing import Dict, List
from urllib.parse import urlparse

from .models import Page, Parameter, Section

logger = logging.getLogger(__name__)


class DocumentAssembler:
    """Assembles parsed pages into a searchable parameter reference document."""

    def organize_by_component(self, pages: List[Page]) -> Dict[str, List[Page]]:
        """
        Organize pages by Nav2 component based on URL structure.

        Returns dict mapping component names to their pages.
        """
        components: Dict[str, List[Page]] = {}

        for page in pages:
            parsed = urlparse(page.url)
            path_parts = parsed.path.strip("/").split("/")

            # Extract component name from URL path
            # e.g., /configuration/packages/configuring-amcl.html -> "AMCL"
            if len(path_parts) >= 3:
                # Get the last meaningful part
                component = path_parts[-1]
                component = component.replace(".html", "")
                component = component.replace("configuring-", "")
                component = component.replace("-", " ").title()
            else:
                component = "General"

            if component not in components:
                components[component] = []
            components[component].append(page)

        return components

    def format_parameter(self, param: Parameter) -> str:
        """Format a single parameter as markdown."""
        lines = []

        lines.append(f"- **`{param.name}`**")

        if param.type and param.type != "N/A":
            lines.append(f"  - Type: `{param.type}`")

        if param.default:
            lines.append(f"  - Default: `{param.default}`")

        if param.description:
            # Clean and truncate description
            desc = param.description.replace("\n", " ").strip()
            if len(desc) > 200:
                desc = desc[:197] + "..."
            lines.append(f"  - {desc}")

        return "\n".join(lines)

    def format_page_parameters(self, page: Page) -> str:
        """Format all parameters from a page."""
        if not page.parameters:
            return ""

        lines = []
        lines.append(f"### {page.title}")
        lines.append(f"*Source: {page.url}*")
        lines.append("")

        for param in sorted(page.parameters, key=lambda p: p.name.lower()):
            lines.append(self.format_parameter(param))
            lines.append("")

        return "\n".join(lines)

    def generate_toc(self, components: Dict[str, List[Page]]) -> str:
        """Generate table of contents."""
        lines = ["## Table of Contents", ""]

        for component in sorted(components.keys()):
            anchor = component.lower().replace(" ", "-")
            lines.append(f"- [{component}](#{anchor})")

        lines.append("")
        return "\n".join(lines)

    def generate_header(self) -> str:
        """Generate document header."""
        timestamp = datetime.now().strftime("%Y-%m-%d")

        return f"""# Nav2 Configuration Parameters Reference

*Generated: {timestamp}*
*Source: https://docs.nav2.org/configuration/*

This document contains all tunable parameters for the Nav2 navigation stack.
Use Ctrl+F / Cmd+F to search for specific parameters.

---

"""

    def assemble_document(
        self,
        pages: List[Page],
        include_toc: bool = True,
        include_metadata: bool = True,
    ) -> str:
        """
        Assemble the final parameter reference document.

        Args:
            pages: List of parsed Page objects with parameters
            include_toc: Whether to include table of contents
            include_metadata: Whether to include header metadata

        Returns:
            Complete markdown document as string
        """
        parts = []

        # Add header
        if include_metadata:
            parts.append(self.generate_header())

        # Organize pages by component
        components = self.organize_by_component(pages)

        # Add TOC
        if include_toc:
            parts.append(self.generate_toc(components))
            parts.append("---\n")

        # Count total parameters
        total_params = sum(len(p.parameters) for p in pages)
        parts.append(f"**Total: {len(pages)} components, {total_params} parameters**\n")
        parts.append("---\n")

        # Add each component section
        for component in sorted(components.keys()):
            parts.append(f"## {component}\n")

            for page in sorted(components[component], key=lambda p: p.title):
                page_content = self.format_page_parameters(page)
                if page_content:
                    parts.append(page_content)

            parts.append("---\n")

        document = "\n".join(parts)

        # Final cleanup
        document = self._cleanup(document)

        logger.info(f"Assembled document: {len(pages)} pages, {total_params} parameters")
        return document

    def _cleanup(self, document: str) -> str:
        """Clean up the final document."""
        import re

        # Remove excessive blank lines
        document = re.sub(r"\n{4,}", "\n\n\n", document)

        # Ensure ends with newline
        return document.strip() + "\n"

    def save_document(self, document: str, output_path: str) -> None:
        """Save the document to a file."""
        import os
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

        with open(output_path, "w", encoding="utf-8") as f:
            f.write(document)

        size_kb = len(document.encode("utf-8")) / 1024
        logger.info(f"Saved document to {output_path} ({size_kb:.1f} KB)")

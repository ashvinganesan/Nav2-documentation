"""Content parser module for extracting Nav2 configuration parameters from HTML."""

import logging
import re
from typing import List, Optional, Tuple

from bs4 import BeautifulSoup, Tag

from .models import Page, Parameter

logger = logging.getLogger(__name__)


class ContentParser:
    """Extracts configuration parameters from Nav2 documentation HTML pages."""

    def __init__(
        self,
        content_selector: str = "div.document",
        exclude_selectors: Optional[List[str]] = None,
    ):
        self.content_selector = content_selector
        self.exclude_selectors = exclude_selectors or [
            "div.sphinxsidebar",
            "div.related",
            "div.footer",
            "nav",
            "div.headerlink",
        ]

    def clean_html(self, html: str) -> BeautifulSoup:
        """Parse HTML and remove unwanted elements."""
        soup = BeautifulSoup(html, "lxml")

        # Remove excluded elements
        for selector in self.exclude_selectors:
            for element in soup.select(selector):
                element.decompose()

        # Remove script and style tags
        for tag in soup.find_all(["script", "style"]):
            tag.decompose()

        # Remove headerlink permalinks
        for link in soup.find_all("a", class_="headerlink"):
            link.decompose()

        # Remove code blocks containing XML/BehaviorTree content
        for pre in soup.find_all("pre"):
            text = pre.get_text()
            if "<BehaviorTree" in text or "<root" in text or "main_tree_to_execute" in text:
                pre.decompose()

        # Remove literal blocks with XML
        for literal in soup.find_all("div", class_="highlight"):
            text = literal.get_text()
            if "<BehaviorTree" in text or "<root" in text or "<?xml" in text:
                literal.decompose()

        return soup

    def extract_title(self, soup: BeautifulSoup) -> str:
        """Extract the page title."""
        h1 = soup.find("h1")
        if h1:
            return h1.get_text(strip=True)

        title = soup.find("title")
        if title:
            text = title.get_text(strip=True)
            text = re.sub(r"\s*[—-]\s*Nav2.*$", "", text)
            return text

        return "Untitled"

    def extract_content(self, soup: BeautifulSoup) -> Optional[Tag]:
        """Extract the main documentation content area."""
        content = soup.select_one(self.content_selector)

        if content is None:
            for selector in ["main", "article", "div.body", "div.content"]:
                content = soup.select_one(selector)
                if content:
                    break

        return content

    def _extract_param_info(self, text: str) -> Tuple[str, Optional[str], str]:
        """
        Extract type, default value, and clean description from parameter text.

        Returns: (type, default, description)
        """
        param_type = "N/A"
        default = None
        description = text

        # Nav2 docs often use format: "(Type, Default: value) Description"
        # or just describe type and default inline

        # Pattern for type extraction
        type_patterns = [
            r"\(([^,\)]+),",  # (Type, ...)
            r"[Tt]ype:\s*`?([^`,\n\)]+)`?",  # Type: value
            r"\btype\s+is\s+`?(\w+)`?",  # type is X
        ]
        for pattern in type_patterns:
            match = re.search(pattern, text)
            if match:
                param_type = match.group(1).strip()
                break

        # Pattern for default value extraction
        default_patterns = [
            r"[Dd]efault:\s*`?([^`\n\)]+)`?",  # Default: value
            r",\s*[Dd]efault\s*[=:]\s*`?([^`\n\)]+)`?",  # , Default = value
            r"\([^,]+,\s*([^)]+)\)",  # (Type, default_value)
        ]
        for pattern in default_patterns:
            match = re.search(pattern, text)
            if match:
                default = match.group(1).strip()
                # Clean up common artifacts
                default = re.sub(r"^\s*[Dd]efault\s*[=:]\s*", "", default)
                break

        # Clean description - remove type/default info for cleaner output
        description = re.sub(r"\([^)]*[Tt]ype[^)]*\)", "", description)
        description = re.sub(r"[Tt]ype:\s*`?[^`,\n]+`?[,.]?\s*", "", description)
        description = re.sub(r"[Dd]efault:\s*`?[^`\n]+`?[,.]?\s*", "", description)
        description = description.strip()

        return param_type, default, description

    def extract_parameters(self, soup: BeautifulSoup) -> List[Parameter]:
        """Extract parameter definitions from the page."""
        parameters = []
        seen_names = set()

        # Method 1: Look for definition lists (dl > dt/dd pairs)
        for dl in soup.find_all("dl"):
            # Skip if this looks like a nav menu or toctree
            if dl.get("class") and any(c in str(dl.get("class")) for c in ["toctree", "nav"]):
                continue

            dt_elements = dl.find_all("dt", recursive=False)
            dd_elements = dl.find_all("dd", recursive=False)

            for dt, dd in zip(dt_elements, dd_elements):
                param = self._parse_dl_parameter(dt, dd)
                if param and param.name not in seen_names:
                    # Filter out non-parameter entries
                    if self._is_valid_parameter(param):
                        parameters.append(param)
                        seen_names.add(param.name)

        # Method 2: Look for bullet lists with parameter patterns
        for ul in soup.find_all(["ul", "ol"]):
            for li in ul.find_all("li", recursive=False):
                param = self._parse_list_parameter(li)
                if param and param.name not in seen_names:
                    if self._is_valid_parameter(param):
                        parameters.append(param)
                        seen_names.add(param.name)

        # Method 3: Look for tables with parameter info
        for table in soup.find_all("table"):
            params = self._parse_parameter_table(table)
            for param in params:
                if param.name not in seen_names:
                    parameters.append(param)
                    seen_names.add(param.name)

        return parameters

    def _is_valid_parameter(self, param: Parameter) -> bool:
        """Check if this looks like a real configuration parameter."""
        name = param.name.lower()

        # Skip entries that are clearly not parameters
        skip_patterns = [
            "example", "note", "warning", "see also", "deprecated",
            "overview", "description", "usage", "http", "https",
        ]
        if any(p in name for p in skip_patterns):
            return False

        # Parameters typically use snake_case or have dots
        if re.match(r"^[a-z][a-z0-9_\.]*$", name):
            return True

        # Or they might be in code formatting
        if param.name.startswith("`") or "." in param.name:
            return True

        # Short single words might be params
        if len(name) < 30 and "_" in name:
            return True

        return len(param.description) > 10

    def _parse_dl_parameter(self, dt: Tag, dd: Tag) -> Optional[Parameter]:
        """Parse a parameter from dt/dd elements with nested structure."""
        name_text = dt.get_text(strip=True)

        if not name_text or len(name_text) > 100:
            return None

        # Clean the name
        name = name_text.strip("`").strip()
        name = re.sub(r"\s*\(.*\)\s*$", "", name).strip()

        if not name:
            return None

        # Try different Nav2 documentation structures:

        # Structure 1 (most common): Table with Type/Default, then dl with Description
        nested_table = dd.find("table")
        if nested_table:
            param_type, default, description = self._parse_nested_table(dd, nested_table)
            if param_type != "N/A" or description:
                return Parameter(
                    name=name,
                    type=param_type,
                    default=default,
                    description=description[:500] if description else "",
                )

        # Structure 2: Nested dl with Type/Default/Description as dt/dd pairs (older format)
        nested_dl = dd.find("dl")
        if nested_dl:
            param_type, default, description = self._parse_nested_dl(nested_dl)
            if param_type != "N/A" or description:
                return Parameter(
                    name=name,
                    type=param_type,
                    default=default,
                    description=description[:500] if description else "",
                )

        # Fallback: plain text extraction
        desc_text = dd.get_text(strip=True)
        param_type, default, description = self._extract_param_info(desc_text)

        return Parameter(
            name=name,
            type=param_type,
            default=default,
            description=description[:500] if description else "",
        )

    def _parse_nested_dl(self, dl: Tag) -> tuple:
        """Parse Nav2's nested dl structure with Type/Default/Description."""
        param_type = "N/A"
        default = None
        description = ""

        dt_elements = dl.find_all("dt", recursive=False)
        dd_elements = dl.find_all("dd", recursive=False)

        for dt_elem, dd_elem in zip(dt_elements, dd_elements):
            label = dt_elem.get_text(strip=True).lower()
            value = dd_elem.get_text(strip=True)

            if "type" in label:
                param_type = value
            elif "default" in label:
                default = value if value else None
            elif "description" in label:
                description = value

        return param_type, default, description

    def _parse_nested_table(self, dd: Tag, table: Tag) -> tuple:
        """Parse table inside dd element with Type/Default in header row."""
        param_type = "N/A"
        default = None
        description = ""

        rows = table.find_all("tr")
        if len(rows) >= 2:
            # First row: headers (Type, Default)
            headers = [cell.get_text(strip=True).lower() for cell in rows[0].find_all(["td", "th", "p"])]
            # Second row: values
            values = [cell.get_text(strip=True) for cell in rows[1].find_all(["td", "th", "p"])]

            for i, header in enumerate(headers):
                if i < len(values):
                    if "type" in header:
                        param_type = values[i]
                    elif "default" in header:
                        default = values[i] if values[i] else None

        # Nav2 structure: Description is in a <dl> after the table
        # <dl class="simple"><dt>Description</dt><dd><p>...</p></dd></dl>
        desc_dl = dd.find("dl", class_="simple")
        if desc_dl:
            desc_dd = desc_dl.find("dd")
            if desc_dd:
                description = desc_dd.get_text(strip=True)

        # Fallback: look for <p> after table
        if not description:
            for sibling in table.find_next_siblings():
                if sibling.name == "p":
                    description = sibling.get_text(strip=True)
                    break
                elif sibling.name == "dl":
                    desc_dd = sibling.find("dd")
                    if desc_dd:
                        description = desc_dd.get_text(strip=True)
                        break

        return param_type, default, description

    def _parse_list_parameter(self, li: Tag) -> Optional[Parameter]:
        """Parse a parameter from a list item."""
        text = li.get_text(strip=True)

        # Look for patterns like: `param_name`: description
        # or: **param_name**: description
        patterns = [
            r"^`([^`]+)`\s*[:\-–]\s*(.+)$",
            r"^\*\*([^*]+)\*\*\s*[:\-–]\s*(.+)$",
            r"^([a-z_][a-z0-9_\.]*)\s*[:\-–]\s*(.+)$",
        ]

        for pattern in patterns:
            match = re.match(pattern, text, re.IGNORECASE | re.DOTALL)
            if match:
                name = match.group(1).strip()
                desc_text = match.group(2).strip()

                if len(name) > 50 or len(name) < 2:
                    continue

                param_type, default, description = self._extract_param_info(desc_text)

                return Parameter(
                    name=name,
                    type=param_type,
                    default=default,
                    description=description[:500] if description else "",
                )

        return None

    def _parse_parameter_table(self, table: Tag) -> List[Parameter]:
        """Parse parameters from an HTML table."""
        parameters = []
        rows = table.find_all("tr")

        if len(rows) < 2:
            return parameters

        # Identify columns from header
        header_row = rows[0]
        headers = [th.get_text(strip=True).lower() for th in header_row.find_all(["th", "td"])]

        name_idx = type_idx = default_idx = desc_idx = None

        for i, header in enumerate(headers):
            if "name" in header or "parameter" in header or "param" in header:
                name_idx = i
            elif "type" in header:
                type_idx = i
            elif "default" in header:
                default_idx = i
            elif "description" in header or "desc" in header:
                desc_idx = i

        if name_idx is None:
            return parameters

        # Parse data rows
        for row in rows[1:]:
            cells = row.find_all(["td", "th"])
            if len(cells) <= name_idx:
                continue

            name = cells[name_idx].get_text(strip=True)
            if not name or len(name) > 100:
                continue

            param_type = "N/A"
            if type_idx is not None and len(cells) > type_idx:
                param_type = cells[type_idx].get_text(strip=True) or "N/A"

            default = None
            if default_idx is not None and len(cells) > default_idx:
                default = cells[default_idx].get_text(strip=True) or None

            description = ""
            if desc_idx is not None and len(cells) > desc_idx:
                description = cells[desc_idx].get_text(strip=True)

            parameters.append(Parameter(
                name=name.strip("`"),
                type=param_type,
                default=default,
                description=description[:500] if description else "",
            ))

        return parameters

    def extract_subsections(self, soup: BeautifulSoup) -> List[str]:
        """Extract subsection titles from the page."""
        subsections = []

        for heading in soup.find_all(["h2", "h3", "h4"]):
            text = heading.get_text(strip=True)
            if text and len(text) < 100:
                subsections.append(text)

        return subsections

    def parse_page(self, url: str, html: str) -> Page:
        """Parse a complete page and return a Page object."""
        soup = self.clean_html(html)

        title = self.extract_title(soup)
        content_element = self.extract_content(soup)

        parameters = []
        subsections = []
        content_html = ""

        if content_element:
            parameters = self.extract_parameters(content_element)
            subsections = self.extract_subsections(content_element)
            # We don't need the full HTML content anymore - just parameters
            content_html = ""

        logger.debug(f"Parsed: {title} - {len(parameters)} parameters")

        return Page(
            url=url,
            title=title,
            content=content_html,
            parameters=parameters,
            subsections=subsections,
        )

    def parse_all_pages(self, pages: dict) -> List[Page]:
        """Parse all scraped pages."""
        parsed_pages = []

        for url, html in pages.items():
            try:
                page = self.parse_page(url, html)
                # Only include pages with parameters
                if page.parameters:
                    parsed_pages.append(page)
                    logger.info(f"Parsed: {page.title} ({len(page.parameters)} parameters)")
                else:
                    logger.debug(f"Skipped (no params): {page.title}")
            except Exception as e:
                logger.error(f"Error parsing {url}: {e}")

        logger.info(f"Parsed {len(parsed_pages)} pages with parameters")
        return parsed_pages

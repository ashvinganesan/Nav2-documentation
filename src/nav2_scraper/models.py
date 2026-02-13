"""Data models for Nav2 documentation scraper."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Parameter:
    """Represents a configuration parameter from Nav2 documentation."""
    name: str
    type: str
    default: Optional[str]
    description: str


@dataclass
class Page:
    """Represents a scraped documentation page."""
    url: str
    title: str
    content: str
    parameters: List[Parameter] = field(default_factory=list)
    subsections: List[str] = field(default_factory=list)


@dataclass
class Section:
    """Represents a hierarchical section in the assembled document."""
    title: str
    level: int
    content: str
    pages: List[Page] = field(default_factory=list)
    subsections: List["Section"] = field(default_factory=list)

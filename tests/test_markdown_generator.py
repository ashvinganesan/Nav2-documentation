"""Tests for the markdown generator module."""

import sys
from pathlib import Path

import pytest

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from nav2_scraper.markdown_generator import MarkdownGenerator
from nav2_scraper.models import Parameter, Page


class TestMarkdownGenerator:
    """Tests for MarkdownGenerator class."""

    def setup_method(self):
        """Set up test fixtures."""
        self.generator = MarkdownGenerator()

    def test_html_to_markdown_basic(self):
        """Test basic HTML to markdown conversion."""
        html = "<p>Hello <strong>world</strong></p>"
        md = self.generator.html_to_markdown(html)
        assert "Hello" in md
        assert "**world**" in md or "world" in md

    def test_html_to_markdown_headings(self):
        """Test heading conversion."""
        html = "<h1>Title</h1><h2>Subtitle</h2>"
        md = self.generator.html_to_markdown(html)
        assert "#" in md

    def test_sanitize_markdown_removes_extra_newlines(self):
        """Test that excessive newlines are removed."""
        markdown = "Line1\n\n\n\n\n\nLine2"
        result = self.generator.sanitize_markdown(markdown)
        assert "\n\n\n\n" not in result

    def test_create_anchor_simple(self):
        """Test anchor creation from simple text."""
        text = "Hello World"
        anchor = self.generator.create_anchor(text)
        assert anchor == "hello-world"

    def test_create_anchor_special_chars(self):
        """Test anchor creation removes special characters."""
        text = "Hello! World? (Test)"
        anchor = self.generator.create_anchor(text)
        assert "!" not in anchor
        assert "?" not in anchor
        assert "(" not in anchor

    def test_fix_heading_levels(self):
        """Test heading level adjustment."""
        markdown = "# Title\n## Subtitle"
        result = self.generator.fix_heading_levels(markdown, base_level=2)
        assert "##" in result  # h1 becomes h2
        assert "###" in result  # h2 becomes h3

    def test_format_parameters(self):
        """Test parameter formatting."""
        params = [
            Parameter(
                name="test_param",
                type="string",
                default="default_value",
                description="A test parameter",
            )
        ]
        result = self.generator.format_parameters(params)
        assert "`test_param`" in result
        assert "`string`" in result
        assert "`default_value`" in result

    def test_format_parameters_empty(self):
        """Test empty parameter list returns empty string."""
        result = self.generator.format_parameters([])
        assert result == ""


class TestPageToMarkdown:
    """Tests for page to markdown conversion."""

    def setup_method(self):
        """Set up test fixtures."""
        self.generator = MarkdownGenerator()

    def test_page_to_markdown_basic(self):
        """Test basic page to markdown conversion."""
        page = Page(
            url="http://example.com/test",
            title="Test Page",
            content="<p>Test content</p>",
            parameters=[],
            subsections=[],
        )
        result = self.generator.page_to_markdown(page)
        assert "Test Page" in result
        assert "http://example.com/test" in result
        assert "Test content" in result

    def test_page_to_markdown_with_parameters(self):
        """Test page with parameters."""
        page = Page(
            url="http://example.com/test",
            title="Test Page",
            content="<p>Content</p>",
            parameters=[
                Parameter("param1", "int", "10", "First param"),
            ],
            subsections=[],
        )
        result = self.generator.page_to_markdown(page)
        assert "param1" in result
        assert "Parameters" in result

"""Tests for the content parser module."""

import sys
from pathlib import Path

import pytest

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from nav2_scraper.parser import ContentParser
from nav2_scraper.models import Parameter


class TestContentParser:
    """Tests for ContentParser class."""

    def setup_method(self):
        """Set up test fixtures."""
        self.parser = ContentParser()

    def test_clean_html_removes_script_tags(self):
        """Test that script tags are removed from HTML."""
        html = "<html><body><script>alert('test')</script><p>Content</p></body></html>"
        soup = self.parser.clean_html(html)
        assert soup.find("script") is None
        assert "Content" in soup.get_text()

    def test_clean_html_removes_style_tags(self):
        """Test that style tags are removed from HTML."""
        html = "<html><body><style>.foo{color:red}</style><p>Content</p></body></html>"
        soup = self.parser.clean_html(html)
        assert soup.find("style") is None

    def test_extract_title_from_h1(self):
        """Test title extraction from h1 tag."""
        html = "<html><body><h1>Test Title</h1><p>Content</p></body></html>"
        soup = self.parser.clean_html(html)
        title = self.parser.extract_title(soup)
        assert title == "Test Title"

    def test_extract_title_from_title_tag(self):
        """Test title extraction from title tag when no h1."""
        html = "<html><head><title>Page Title — Nav2 documentation</title></head><body></body></html>"
        soup = self.parser.clean_html(html)
        title = self.parser.extract_title(soup)
        assert title == "Page Title"

    def test_extract_subsections(self):
        """Test subsection extraction from headings."""
        html = """
        <html><body>
        <h1>Main</h1>
        <h2>Section One</h2>
        <h3>Subsection A</h3>
        <h2>Section Two</h2>
        </body></html>
        """
        soup = self.parser.clean_html(html)
        subsections = self.parser.extract_subsections(soup)
        assert "Section One" in subsections
        assert "Section Two" in subsections
        assert "Subsection A" in subsections

    def test_parse_page_returns_page_object(self):
        """Test that parse_page returns a Page object."""
        html = """
        <html><body>
        <div class="document">
            <h1>Test Page</h1>
            <p>Some content here.</p>
        </div>
        </body></html>
        """
        page = self.parser.parse_page("http://example.com/test", html)
        assert page.url == "http://example.com/test"
        assert page.title == "Test Page"
        assert "Some content here" in page.content


class TestParameterExtraction:
    """Tests for parameter extraction."""

    def setup_method(self):
        """Set up test fixtures."""
        self.parser = ContentParser()

    def test_extract_parameters_from_dl(self):
        """Test parameter extraction from definition list."""
        html = """
        <html><body>
        <dl>
            <dt>parameter_name</dt>
            <dd>Type: string. Default: value. Description of the parameter.</dd>
        </dl>
        </body></html>
        """
        soup = self.parser.clean_html(html)
        params = self.parser.extract_parameters(soup)
        assert len(params) == 1
        assert params[0].name == "parameter_name"

    def test_extract_parameters_from_table(self):
        """Test parameter extraction from table."""
        html = """
        <html><body>
        <table>
            <tr><th>Name</th><th>Type</th><th>Default</th><th>Description</th></tr>
            <tr><td>param1</td><td>int</td><td>10</td><td>First parameter</td></tr>
            <tr><td>param2</td><td>string</td><td>foo</td><td>Second parameter</td></tr>
        </table>
        </body></html>
        """
        soup = self.parser.clean_html(html)
        params = self.parser.extract_parameters(soup)
        assert len(params) == 2
        assert params[0].name == "param1"
        assert params[0].type == "int"
        assert params[0].default == "10"

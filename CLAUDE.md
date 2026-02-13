# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Python-based web scraper that extracts Nav2 (ROS 2 Navigation) documentation from docs.nav2.org and generates a comprehensive, searchable markdown file. The goal is to consolidate Nav2's configuration parameters across many packages into a single well-structured document.

## Build and Run Commands

```bash
# Install dependencies
pip install -r requirements.txt

# Run the scraper
python scripts/run_scraper.py

# Validate output
python scripts/validate_output.py data/output/nav2_documentation_complete.md

# Run tests
pytest tests/

# Run a single test file
pytest tests/test_scraper.py

# Linting and formatting
black src/
flake8 src/
```

## Architecture

The project consists of four core modules in `src/nav2_scraper/`:

1. **scraper.py** - Crawls docs.nav2.org using BFS, extracts links, handles rate limiting (1 req/sec)
2. **parser.py** - Extracts content from HTML, removes nav/footer elements, parses parameter tables
3. **markdown_generator.py** - Converts HTML to markdown, preserves code blocks and tables
4. **assembler.py** - Organizes sections hierarchically, generates TOC, creates cross-references

## Data Flow

```
Raw HTML (data/raw/) → Parsed Content (data/processed/) → Final Markdown (data/output/)
```

## Key Data Models

- `Parameter`: name, type, default, description
- `Page`: url, title, content, parameters, subsections
- `Section`: title, level, content, pages, subsections (recursive)

## Configuration

Config lives in `config/scraper_config.yaml`:
- `scraper.start_url`: Entry point (https://docs.nav2.org/configuration/index.html)
- `scraper.rate_limit_seconds`: Delay between requests (default 1.0)
- `parser.content_selector`: Main content div (`div.document`)
- `parser.exclude_selectors`: Elements to remove (sidebar, footer, related)

## Key Dependencies

- `requests` / `beautifulsoup4` / `lxml` - Web scraping and HTML parsing
- `markdownify` - HTML to markdown conversion
- `pyyaml` - Configuration handling
- `tqdm` - Progress indicators

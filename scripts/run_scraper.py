#!/usr/bin/env python3
"""Main entry point for running the Nav2 configuration parameter scraper."""

import argparse
import logging
import os
import sys
from pathlib import Path

import yaml

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from nav2_scraper.scraper import Nav2Scraper
from nav2_scraper.parser import ContentParser
from nav2_scraper.assembler import DocumentAssembler


def setup_logging(verbose: bool = False) -> None:
    """Configure logging."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(),
        ],
    )


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    try:
        with open(config_path, "r") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        return {}


def main():
    parser = argparse.ArgumentParser(
        description="Scrape Nav2 configuration parameters and generate searchable markdown"
    )
    parser.add_argument(
        "-c", "--config",
        default="config/scraper_config.yaml",
        help="Path to configuration file",
    )
    parser.add_argument(
        "-o", "--output",
        default="data/output/nav2_parameters.md",
        help="Output file path",
    )
    parser.add_argument(
        "--save-raw",
        action="store_true",
        help="Save raw HTML files for debugging",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose logging",
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        default=5,
        help="Max crawl depth (default: 5)",
    )

    args = parser.parse_args()
    setup_logging(args.verbose)

    logger = logging.getLogger(__name__)
    logger.info("Nav2 Configuration Parameter Scraper")
    logger.info("=" * 50)

    # Load configuration
    config = load_config(args.config)
    scraper_config = config.get("scraper", {})
    parser_config = config.get("parser", {})
    output_config = config.get("output", {})

    # Initialize scraper - only crawl /configuration/ pages
    scraper = Nav2Scraper(
        base_domain=scraper_config.get("base_domain", "docs.nav2.org"),
        allowed_path_prefix="/configuration/",
        rate_limit=scraper_config.get("rate_limit_seconds", 0.5),
        timeout=scraper_config.get("timeout_seconds", 30),
        max_retries=scraper_config.get("max_retries", 3),
        user_agent=scraper_config.get("user_agent", "Nav2DocScraper/1.0"),
    )

    content_parser = ContentParser(
        content_selector=parser_config.get("content_selector", "div.document"),
        exclude_selectors=parser_config.get("exclude_selectors"),
    )

    assembler = DocumentAssembler()

    # Step 1: Scrape configuration pages only
    start_url = "https://docs.nav2.org/configuration/index.html"
    logger.info(f"Crawling: {start_url}")
    logger.info(f"Max depth: {args.max_depth}")
    logger.info("Only scraping /configuration/ pages...")

    raw_pages = scraper.crawl_documentation(start_url, max_depth=args.max_depth)
    logger.info(f"Scraped {len(raw_pages)} configuration pages")

    if not raw_pages:
        logger.error("No pages were scraped. Exiting.")
        sys.exit(1)

    # Save raw HTML if requested
    if args.save_raw:
        scraper.save_raw_html(raw_pages, "data/raw")

    # Step 2: Parse pages and extract parameters
    logger.info("Extracting parameters...")
    parsed_pages = content_parser.parse_all_pages(raw_pages)

    if not parsed_pages:
        logger.error("No parameters found. Exiting.")
        sys.exit(1)

    total_params = sum(len(p.parameters) for p in parsed_pages)
    logger.info(f"Found {total_params} parameters across {len(parsed_pages)} pages")

    # Step 3: Assemble document
    logger.info("Generating document...")
    document = assembler.assemble_document(
        parsed_pages,
        include_toc=output_config.get("include_toc", True),
        include_metadata=output_config.get("include_metadata", True),
    )

    # Step 4: Save output
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    assembler.save_document(document, args.output)

    # Print summary
    file_size = os.path.getsize(args.output)
    logger.info("=" * 50)
    logger.info(f"Done! Output: {args.output}")
    logger.info(f"Size: {file_size / 1024:.1f} KB")
    logger.info(f"Parameters: {total_params}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

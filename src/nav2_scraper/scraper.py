"""Web scraper module for crawling Nav2 documentation."""

import logging
import time
from typing import Dict, List, Optional, Set
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup
from tqdm import tqdm

logger = logging.getLogger(__name__)


class Nav2Scraper:
    """Crawls and downloads pages from docs.nav2.org."""

    def __init__(
        self,
        base_domain: str = "docs.nav2.org",
        allowed_path_prefix: str = "/configuration/",
        rate_limit: float = 1.0,
        timeout: int = 30,
        max_retries: int = 3,
        user_agent: str = "Nav2DocScraper/1.0",
    ):
        self.base_domain = base_domain
        self.allowed_path_prefix = allowed_path_prefix
        self.rate_limit = rate_limit
        self.timeout = timeout
        self.max_retries = max_retries
        self.session = requests.Session()
        self.session.headers.update({"User-Agent": user_agent})
        self.visited: Set[str] = set()
        self.last_request_time: float = 0

    def _wait_for_rate_limit(self) -> None:
        """Enforce rate limiting between requests."""
        elapsed = time.time() - self.last_request_time
        if elapsed < self.rate_limit:
            time.sleep(self.rate_limit - elapsed)
        self.last_request_time = time.time()

    def get_page(self, url: str) -> Optional[str]:
        """Fetch a single page with retry logic."""
        for attempt in range(self.max_retries):
            try:
                self._wait_for_rate_limit()
                response = self.session.get(url, timeout=self.timeout)
                response.raise_for_status()
                return response.text
            except requests.exceptions.HTTPError as e:
                if response.status_code == 429:
                    # Rate limited - wait longer
                    wait_time = (attempt + 1) * 5
                    logger.warning(f"Rate limited on {url}, waiting {wait_time}s")
                    time.sleep(wait_time)
                elif response.status_code == 404:
                    logger.warning(f"Page not found: {url}")
                    return None
                else:
                    logger.error(f"HTTP error on {url}: {e}")
            except requests.exceptions.RequestException as e:
                logger.error(f"Request error on {url} (attempt {attempt + 1}): {e}")
                if attempt < self.max_retries - 1:
                    time.sleep(2 ** attempt)

        logger.error(f"Failed to fetch {url} after {self.max_retries} attempts")
        return None

    def is_nav2_doc_url(self, url: str) -> bool:
        """Check if URL is a valid Nav2 configuration documentation page."""
        parsed = urlparse(url)

        # Must be on the correct domain
        if parsed.netloc and parsed.netloc != self.base_domain:
            return False

        # Must be under the allowed path prefix (e.g., /configuration/)
        if not parsed.path.startswith(self.allowed_path_prefix):
            return False

        # Skip non-HTML resources
        path = parsed.path.lower()
        skip_extensions = (".png", ".jpg", ".jpeg", ".gif", ".svg", ".pdf", ".zip", ".gz")
        if any(path.endswith(ext) for ext in skip_extensions):
            return False

        # Skip external links and anchors-only
        if parsed.scheme and parsed.scheme not in ("http", "https", ""):
            return False

        return True

    def extract_links(self, html: str, base_url: str) -> List[str]:
        """Extract all internal documentation links from HTML."""
        soup = BeautifulSoup(html, "lxml")
        links = []

        for anchor in soup.find_all("a", href=True):
            href = anchor["href"]

            # Skip anchors within the same page
            if href.startswith("#"):
                continue

            # Resolve relative URLs
            full_url = urljoin(base_url, href)

            # Remove fragment
            parsed = urlparse(full_url)
            clean_url = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
            if parsed.path.endswith("/"):
                clean_url = clean_url.rstrip("/") + "/index.html"

            if self.is_nav2_doc_url(clean_url) and clean_url not in self.visited:
                links.append(clean_url)

        return list(set(links))

    def crawl_documentation(
        self, start_url: str, max_depth: int = 10
    ) -> Dict[str, str]:
        """
        Crawl documentation using breadth-first search.

        Args:
            start_url: The starting URL for crawling
            max_depth: Maximum link depth to crawl

        Returns:
            Dictionary mapping URLs to their HTML content
        """
        pages: Dict[str, str] = {}
        queue: List[tuple] = [(start_url, 0)]
        self.visited.clear()

        # Create progress bar
        pbar = tqdm(desc="Scraping pages", unit="pages")

        while queue:
            url, depth = queue.pop(0)

            if url in self.visited or depth > max_depth:
                continue

            self.visited.add(url)
            logger.info(f"Scraping (depth={depth}): {url}")

            html = self.get_page(url)
            if html is None:
                continue

            pages[url] = html
            pbar.update(1)

            if depth < max_depth:
                new_links = self.extract_links(html, url)
                for link in new_links:
                    if link not in self.visited:
                        queue.append((link, depth + 1))

        pbar.close()
        logger.info(f"Crawling complete. Scraped {len(pages)} pages.")
        return pages

    def save_raw_html(self, pages: Dict[str, str], output_dir: str) -> None:
        """Save raw HTML files to disk for caching/debugging."""
        import hashlib
        import os

        os.makedirs(output_dir, exist_ok=True)

        for url, html in pages.items():
            # Create filename from URL hash
            url_hash = hashlib.md5(url.encode()).hexdigest()[:12]
            parsed = urlparse(url)
            path_part = parsed.path.replace("/", "_").strip("_") or "index"
            filename = f"{path_part}_{url_hash}.html"

            filepath = os.path.join(output_dir, filename)
            with open(filepath, "w", encoding="utf-8") as f:
                f.write(f"<!-- URL: {url} -->\n")
                f.write(html)

        logger.info(f"Saved {len(pages)} HTML files to {output_dir}")

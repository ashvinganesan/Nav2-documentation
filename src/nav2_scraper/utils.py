"""Utility functions for the Nav2 documentation scraper."""

import logging
import os
from pathlib import Path
from typing import Optional

import yaml


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent.parent


def load_config(config_path: Optional[str] = None) -> dict:
    """
    Load configuration from YAML file.

    Args:
        config_path: Path to config file. If None, uses default location.

    Returns:
        Configuration dictionary
    """
    if config_path is None:
        config_path = get_project_root() / "config" / "scraper_config.yaml"

    config_path = Path(config_path)

    if not config_path.exists():
        logging.warning(f"Config file not found: {config_path}, using defaults")
        return {}

    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def setup_logging(
    level: int = logging.INFO,
    log_file: Optional[str] = None,
) -> None:
    """
    Configure logging for the application.

    Args:
        level: Logging level
        log_file: Optional path to log file
    """
    handlers = [logging.StreamHandler()]

    if log_file:
        os.makedirs(os.path.dirname(log_file) or ".", exist_ok=True)
        handlers.append(logging.FileHandler(log_file))

    logging.basicConfig(
        level=level,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=handlers,
    )


def ensure_dir(path: str) -> str:
    """Ensure a directory exists, creating it if necessary."""
    os.makedirs(path, exist_ok=True)
    return path

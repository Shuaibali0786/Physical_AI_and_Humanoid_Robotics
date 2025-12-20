"""
Utility functions for RAG Agent
"""
import time
import logging
import re
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse


def clean_text(text: str) -> str:
    """
    Clean and normalize text content

    Args:
        text: Raw text to clean

    Returns:
        Cleaned text with normalized whitespace
    """
    if not text:
        return ""

    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    return text


def validate_url(url: str) -> bool:
    """
    Validate that a string is a properly formatted URL

    Args:
        url: URL string to validate

    Returns:
        True if valid URL, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def filter_urls_by_domain(urls: List[str], base_url: str) -> List[str]:
    """
    Filter a list of URLs to only include those from the same domain as the base URL

    Args:
        urls: List of URLs to filter
        base_url: Base URL to compare against

    Returns:
        List of URLs from the same domain
    """
    base_domain = urlparse(base_url).netloc

    filtered_urls = []
    for url in urls:
        try:
            parsed = urlparse(url)
            if parsed.netloc == base_domain:
                filtered_urls.append(url)
        except Exception:
            # Skip invalid URLs
            continue

    return filtered_urls


def retry_on_failure(max_retries: int = 3, delay: float = 1.0):
    """
    Decorator to retry a function on failure

    Args:
        max_retries: Maximum number of retry attempts
        delay: Delay between retries in seconds
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            last_exception = None
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries - 1:
                        time.sleep(delay * (2 ** attempt))  # Exponential backoff
                    else:
                        break
            raise last_exception
        return wrapper
    return decorator


def format_logger(logger_name: str, level: int = logging.INFO) -> logging.Logger:
    """
    Create and configure a logger with standard formatting

    Args:
        logger_name: Name for the logger
        level: Logging level to use

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(logger_name)
    logger.setLevel(level)

    # Prevent adding multiple handlers if logger already exists
    if logger.handlers:
        return logger

    handler = logging.StreamHandler()
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger


def chunk_text_recursive(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
    """
    Recursively split text into chunks of specified size with overlap.

    Args:
        text: The text to chunk
        chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks

    Returns:
        List of text chunks
    """
    if len(text) <= chunk_size:
        return [text]

    # Find a good breaking point (try to break at sentence or paragraph boundaries)
    break_point = chunk_size
    for i in range(chunk_size, max(chunk_size - overlap, 0), -1):
        if i < len(text):
            if text[i] in '.!?':
                break_point = i + 1
                break
            elif text[i] == '\n':
                break_point = i
                break
            elif text[i] == ' ':
                break_point = i
                break

    # If we couldn't find a good break point, just break at chunk_size
    if break_point >= len(text):
        break_point = chunk_size

    first_chunk = text[:break_point]
    remaining_text = text[break_point - overlap:]  # Apply overlap

    chunks = [first_chunk]
    if remaining_text.strip():
        chunks.extend(chunk_text_recursive(remaining_text, chunk_size, overlap))

    return chunks
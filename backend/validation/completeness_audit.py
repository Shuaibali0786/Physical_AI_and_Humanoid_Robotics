"""
Functions for identifying missing or corrupted data in the Qdrant collection
"""
import logging
from typing import List, Dict, Any, Tuple, Set
from qdrant_client import QdrantClient


logger = logging.getLogger(__name__)


def identify_missing_data(original_urls: List[str], stored_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Identify and report any missing or corrupted data in the collection.

    Args:
        original_urls: List of original book URLs
        stored_chunks: List of stored chunks from Qdrant

    Returns:
        List of missing/corrupted data with specific details
    """
    original_url_set = set(original_urls)
    stored_url_set = {chunk.get('url', '') for chunk in stored_chunks if chunk.get('url')}

    missing_urls = list(original_url_set - stored_url_set)
    extra_urls = list(stored_url_set - original_url_set)  # URLs in storage but not in original list

    results = {
        "missing_urls": missing_urls,
        "extra_urls": extra_urls,
        "missing_count": len(missing_urls),
        "extra_count": len(extra_urls),
        "total_original": len(original_urls),
        "total_stored": len(stored_chunks)
    }

    logger.info(f"Completeness audit: {results['missing_count']} missing, {results['extra_count']} extra URLs")
    return results


def add_url_comparison_logic(original_urls: List[str], stored_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Add URL comparison logic to check for missing entries.

    Args:
        original_urls: List of original book URLs
        stored_chunks: List of stored chunks from Qdrant

    Returns:
        Dictionary with comparison results
    """
    return identify_missing_data(original_urls, stored_chunks)


def implement_corrupted_data_detection(chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Implement corrupted data detection.

    Args:
        chunks: List of chunk objects to check for corruption

    Returns:
        List of corrupted chunks with details
    """
    corrupted_chunks = []

    for chunk in chunks:
        issues = []

        # Check for missing required fields
        required_fields = ['id', 'url', 'content', 'payload']
        for field in required_fields:
            if field not in chunk or chunk[field] is None:
                issues.append(f"Missing or null field: {field}")

        # Check for empty content
        if chunk.get('content', '').strip() == '':
            issues.append("Content is empty")

        # Check for invalid payload structure
        payload = chunk.get('payload', {})
        if not isinstance(payload, dict):
            issues.append("Payload is not a dictionary")

        # Check for invalid vector data (if present)
        vector = chunk.get('vector')
        if vector is not None:
            if not isinstance(vector, list):
                issues.append("Vector is not a list")
            else:
                # Check if vector has valid numeric values
                for i, val in enumerate(vector):
                    if not isinstance(val, (int, float)):
                        issues.append(f"Vector contains non-numeric value at index {i}: {val}")
                        break

        if issues:
            corrupted_chunks.append({
                "id": chunk.get('id', 'unknown'),
                "url": chunk.get('url', 'unknown'),
                "issues": issues
            })

    logger.info(f"Detected {len(corrupted_chunks)} potentially corrupted chunks")
    return corrupted_chunks


def add_validation_for_content_completeness(chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Add validation for content completeness.

    Args:
        chunks: List of chunk objects to validate

    Returns:
        Dictionary with completeness validation results
    """
    total_chunks = len(chunks)
    valid_chunks = 0
    incomplete_chunks = 0
    completeness_issues = []

    for chunk in chunks:
        is_complete = True
        issues = []

        # Check if content is substantial
        content = chunk.get('content', '')
        if len(content.strip()) < 10:  # Less than 10 characters is considered incomplete
            is_complete = False
            issues.append(f"Content too short: {len(content)} characters")

        # Check for essential metadata
        title = chunk.get('title', '')
        if not title.strip():
            is_complete = False
            issues.append("Missing or empty title")

        url = chunk.get('url', '')
        if not url.strip():
            is_complete = False
            issues.append("Missing or empty URL")

        if is_complete:
            valid_chunks += 1
        else:
            incomplete_chunks += 1
            completeness_issues.append({
                "id": chunk.get('id', 'unknown'),
                "url": url,
                "issues": issues
            })

    results = {
        "total_chunks": total_chunks,
        "valid_chunks": valid_chunks,
        "incomplete_chunks": incomplete_chunks,
        "completeness_rate": (valid_chunks / total_chunks * 100) if total_chunks > 0 else 0,
        "completeness_issues": completeness_issues
    }

    logger.info(f"Content completeness validation: {results['completeness_rate']:.2f}% complete")
    return results


def create_detailed_issue_reporting(issues: List[Dict[str, Any]]) -> str:
    """
    Create detailed issue reporting for identified problems.

    Args:
        issues: List of issues to report

    Returns:
        Formatted string with detailed issue report
    """
    if not issues:
        return "No issues identified"

    report_lines = ["Detailed Issue Report:", "-" * 20]

    for i, issue in enumerate(issues, 1):
        report_lines.append(f"Issue {i}:")
        for key, value in issue.items():
            report_lines.append(f"  {key}: {value}")
        report_lines.append("")  # Empty line for readability

    return "\n".join(report_lines)
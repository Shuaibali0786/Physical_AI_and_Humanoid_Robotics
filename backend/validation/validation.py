"""
Functions for validating data integrity
"""
import logging
from typing import List, Dict, Any, Tuple
import numpy as np


logger = logging.getLogger(__name__)


def validate_embedding_dimensions(embeddings: List[List[float]], expected_dimension: int = 1024) -> Dict[str, Any]:
    """
    Validate that stored embeddings have correct dimensions and values.

    Args:
        embeddings: List of embedding vectors to validate
        expected_dimension: Expected dimension of the embeddings (default 1024 for Cohere)

    Returns:
        Validation results object with counts of valid/invalid embeddings
    """
    valid_embeddings = 0
    invalid_embeddings = 0
    invalid_details = []

    for i, embedding in enumerate(embeddings):
        is_valid = True
        reasons = []

        # Check dimension
        if len(embedding) != expected_dimension:
            is_valid = False
            reasons.append(f"Wrong dimension: {len(embedding)} instead of {expected_dimension}")

        # Check for null/empty values
        if any(v is None or (isinstance(v, float) and np.isnan(v)) for v in embedding):
            is_valid = False
            reasons.append("Contains null or NaN values")

        # Check if all values are floats
        if not all(isinstance(v, (int, float, np.floating)) for v in embedding):
            is_valid = False
            reasons.append("Contains non-numeric values")

        if is_valid:
            valid_embeddings += 1
        else:
            invalid_embeddings += 1
            invalid_details.append({
                "index": i,
                "reasons": reasons,
                "sample_values": embedding[:5]  # Show first 5 values as sample
            })

    results = {
        "total_embeddings": len(embeddings),
        "valid_embeddings": valid_embeddings,
        "invalid_embeddings": invalid_embeddings,
        "expected_dimension": expected_dimension,
        "valid_percentage": (valid_embeddings / len(embeddings)) * 100 if embeddings else 0,
        "invalid_details": invalid_details
    }

    logger.info(f"Embedding validation completed: {valid_embeddings} valid, {invalid_embeddings} invalid")
    return results


def validate_embedding_values(embedding: List[float]) -> Tuple[bool, List[str]]:
    """
    Add validation for float value ranges in embeddings.

    Args:
        embedding: Single embedding vector to validate

    Returns:
        Tuple of (is_valid, list_of_issues)
    """
    issues = []

    for i, value in enumerate(embedding):
        if not isinstance(value, (int, float, np.floating)):
            issues.append(f"Value at index {i} is not numeric: {type(value)}")
        elif np.isnan(value) or np.isinf(value):
            issues.append(f"Value at index {i} is NaN or infinite: {value}")
        elif not (-2.0 <= value <= 2.0):  # Typical range for embeddings
            # This is just a general check - embeddings can have different ranges depending on the model
            # For now, we'll just log values outside a typical range
            logger.debug(f"Value at index {i} may be outside typical range: {value}")

    return len(issues) == 0, issues


def calculate_embedding_statistics(embeddings: List[List[float]]) -> Dict[str, Any]:
    """
    Create embedding validation summary statistics.

    Args:
        embeddings: List of embedding vectors to analyze

    Returns:
        Dictionary with summary statistics
    """
    if not embeddings:
        return {
            "count": 0,
            "dimension_stats": {},
            "value_range": {},
            "summary": "No embeddings to analyze"
        }

    # Calculate basic statistics
    embedding_count = len(embeddings)
    dimensions = len(embeddings[0]) if embeddings else 0

    # Check dimension consistency
    dimension_consistent = all(len(embedding) == dimensions for embedding in embeddings)

    # Calculate min/max values across all embeddings
    all_values = []
    for embedding in embeddings:
        all_values.extend(embedding)

    if all_values:
        min_value = min(all_values)
        max_value = max(all_values)
        avg_value = sum(all_values) / len(all_values)
    else:
        min_value = max_value = avg_value = 0

    stats = {
        "count": embedding_count,
        "dimensions": dimensions,
        "dimension_consistency": dimension_consistent,
        "value_range": {
            "min": min_value,
            "max": max_value,
            "average": avg_value
        },
        "summary": f"Analyzed {embedding_count} embeddings with {dimensions} dimensions each"
    }

    logger.info(f"Embedding statistics: {stats['summary']}")
    return stats


def validate_chunk_integrity(chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Validate the integrity of stored chunks.

    Args:
        chunks: List of chunk objects with metadata

    Returns:
        Validation results for chunk integrity
    """
    valid_chunks = 0
    invalid_chunks = 0
    invalid_details = []

    for i, chunk in enumerate(chunks):
        is_valid = True
        reasons = []

        # Check required fields
        required_fields = ["id", "url", "title", "content"]
        for field in required_fields:
            if field not in chunk or not chunk[field]:
                is_valid = False
                reasons.append(f"Missing or empty field: {field}")

        # Check content length
        if "content" in chunk and len(chunk["content"]) == 0:
            is_valid = False
            reasons.append("Content is empty")

        # Check URL format (basic check)
        if "url" in chunk and chunk["url"]:
            url = chunk["url"]
            if not url.startswith(("http://", "https://")):
                is_valid = False
                reasons.append(f"Invalid URL format: {url}")

        if is_valid:
            valid_chunks += 1
        else:
            invalid_chunks += 1
            invalid_details.append({
                "id": chunk.get("id", f"index_{i}"),
                "reasons": reasons
            })

    results = {
        "total_chunks": len(chunks),
        "valid_chunks": valid_chunks,
        "invalid_chunks": invalid_chunks,
        "valid_percentage": (valid_chunks / len(chunks)) * 100 if chunks else 0,
        "invalid_details": invalid_details
    }

    logger.info(f"Chunk validation completed: {valid_chunks} valid, {invalid_chunks} invalid")
    return results


def validate_vector_values(embedding: List[float]) -> Tuple[bool, List[str]]:
    """
    Validate individual vector values.

    Args:
        embedding: Single embedding vector to validate

    Returns:
        Tuple of (is_valid, list_of_issues)
    """
    issues = []

    for i, value in enumerate(embedding):
        if not isinstance(value, (int, float, np.floating)):
            issues.append(f"Value at index {i} is not numeric: {type(value)}")
        elif np.isnan(value) or np.isinf(value):
            issues.append(f"Value at index {i} is NaN or infinite: {value}")

    return len(issues) == 0, issues
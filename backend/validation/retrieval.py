"""
Functions for retrieving data from Qdrant
"""
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models


logger = logging.getLogger(__name__)


def retrieve_all_chunks_from_qdrant(client: QdrantClient, collection_name: str) -> List[Dict[str, Any]]:
    """
    Retrieve all stored chunks from the Qdrant collection with their metadata.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to retrieve from

    Returns:
        List of chunk objects with metadata (URL, title, content, etc.)
    """
    try:
        # Check if collection exists
        collection_info = client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' exists, retrieving all points...")
        logger.info(f"Collection points count: {collection_info.points_count}")
    except Exception as e:
        logger.error(f"Collection '{collection_name}' does not exist: {e}")
        return []

    all_chunks = []
    offset = None
    batch_size = 100  # Retrieve in batches to handle large collections

    while True:
        try:
            # Retrieve a batch of points
            points, next_offset = client.scroll(
                collection_name=collection_name,
                limit=batch_size,
                offset=offset,
                with_payload=True,
                with_vectors=True
            )

            # Process the retrieved points
            for point in points:
                chunk_data = {
                    "id": point.id,
                    "vector": point.vector,
                    "payload": point.payload,
                    "url": point.payload.get("url", ""),
                    "title": point.payload.get("title", ""),
                    "content": point.payload.get("content", ""),
                    "chunk_index": point.payload.get("chunk_index", 0),
                    "content_type": point.payload.get("content_type", "text"),
                    "created_at": point.payload.get("created_at", "")
                }
                all_chunks.append(chunk_data)

            # If no more points, break the loop
            if next_offset is None:
                break

            offset = next_offset

        except Exception as e:
            logger.error(f"Error retrieving chunks from Qdrant: {e}")
            break

    logger.info(f"Retrieved {len(all_chunks)} chunks from Qdrant collection '{collection_name}'")
    return all_chunks


def validate_collection_existence(client: QdrantClient, collection_name: str) -> bool:
    """
    Implement collection existence validation.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to check

    Returns:
        True if collection exists, False otherwise
    """
    try:
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' exists")
        return True
    except Exception:
        logger.warning(f"Collection '{collection_name}' does not exist")
        return False


def validate_metadata_completeness(chunk: Dict[str, Any]) -> Tuple[bool, List[str]]:
    """
    Add validation for metadata completeness (URL, title, content).

    Args:
        chunk: A chunk object with metadata

    Returns:
        Tuple of (is_valid, list_of_missing_fields)
    """
    required_fields = ["url", "title", "content"]
    missing_fields = []

    for field in required_fields:
        if not chunk.get(field):
            missing_fields.append(field)

    return len(missing_fields) == 0, missing_fields


def count_and_verify_chunks(chunks: List[Dict[str, Any]]) -> Dict[str, int]:
    """
    Implement chunk counting and verification.

    Args:
        chunks: List of chunk objects to count and verify

    Returns:
        Dictionary with count statistics
    """
    total_count = len(chunks)
    valid_count = 0
    invalid_count = 0

    for chunk in chunks:
        is_valid, _ = validate_metadata_completeness(chunk)
        if is_valid:
            valid_count += 1
        else:
            invalid_count += 1

    stats = {
        "total_chunks": total_count,
        "valid_chunks": valid_count,
        "invalid_chunks": invalid_count,
        "valid_percentage": (valid_count / total_count * 100) if total_count > 0 else 0
    }

    logger.info(f"Chunk validation: {valid_count}/{total_count} ({stats['valid_percentage']:.2f}%) valid")
    return stats


def get_collection_info(client: QdrantClient, collection_name: str) -> Optional[Dict[str, Any]]:
    """
    Get information about the Qdrant collection.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection

    Returns:
        Collection information or None if collection doesn't exist
    """
    try:
        collection_info = client.get_collection(collection_name)
        return {
            "name": collection_info.config.params.vectors.size,
            "vector_size": collection_info.config.params.vectors.size,
            "distance": collection_info.config.params.vectors.distance,
            "points_count": collection_info.points_count,
            "indexed_vectors_count": collection_info.indexed_vectors_count
        }
    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        return None
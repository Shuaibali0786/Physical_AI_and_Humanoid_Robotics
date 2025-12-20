"""
Module for retrieving content from Qdrant
"""
import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.conversions import common_types
from .config import get_settings
from .models import RetrievedChunk
from .utils import retry_on_failure


logger = logging.getLogger(__name__)
settings = get_settings()


def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance using configuration
    """
    if not settings.qdrant_url or not settings.qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in environment variables")

    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        timeout=30
    )

    return client


@retry_on_failure(max_retries=3, delay=1.0)
def retrieve_chunks_from_qdrant(query: str, top_k: int = 5) -> List[RetrievedChunk]:
    """
    Retrieve relevant content chunks from Qdrant based on semantic similarity to the query.

    Args:
        query: The query text to find similar content for
        top_k: Number of top results to return (default 5)

    Returns:
        List of RetrievedChunk objects with content and metadata
    """
    try:
        client = get_qdrant_client()

        # Generate embedding for the query using Cohere
        # In a real implementation, we would call the Cohere API to get an embedding for the query
        # For now, we'll use a mock approach to illustrate the concept
        query_embedding = generate_query_embedding(query)

        # Perform semantic search in Qdrant
        search_results = client.search(
            collection_name=settings.qdrant_collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        retrieved_chunks = []
        for result in search_results:
            payload = result.payload

            chunk = RetrievedChunk(
                id=str(result.id),
                content=payload.get("content", ""),
                score=result.score,
                url=payload.get("url", ""),
                title=payload.get("title", ""),
                metadata=payload.get("metadata", {})
            )

            retrieved_chunks.append(chunk)

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: {query[:50]}...")
        return retrieved_chunks

    except Exception as e:
        logger.error(f"Error retrieving chunks from Qdrant: {e}")
        raise


def generate_query_embedding(query: str) -> List[float]:
    """
    Generate an embedding for the query text using Cohere.

    Args:
        query: The query text to embed

    Returns:
        Embedding vector as a list of floats
    """
    import cohere

    if not settings.cohere_api_key:
        raise ValueError("COHERE_API_KEY must be set in environment variables")

    co = cohere.Client(settings.cohere_api_key)

    try:
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )

        # Return the first (and only) embedding
        return response.embeddings[0]

    except Exception as e:
        logger.error(f"Error generating query embedding: {e}")
        raise


def validate_qdrant_connection() -> bool:
    """
    Validate that we can connect to Qdrant and the collection exists.

    Returns:
        True if connection is successful and collection exists, False otherwise
    """
    try:
        client = get_qdrant_client()

        # Try to get collection info to verify it exists
        client.get_collection(settings.qdrant_collection_name)

        logger.info(f"Successfully connected to Qdrant collection: {settings.qdrant_collection_name}")
        return True

    except Exception as e:
        logger.error(f"Failed to connect to Qdrant collection {settings.qdrant_collection_name}: {e}")
        return False


def get_total_chunks_count() -> int:
    """
    Get the total number of chunks stored in the Qdrant collection.

    Returns:
        Total number of chunks in the collection
    """
    try:
        client = get_qdrant_client()

        collection_info = client.get_collection(settings.qdrant_collection_name)
        return collection_info.points_count

    except Exception as e:
        logger.error(f"Error getting chunks count from Qdrant: {e}")
        return 0
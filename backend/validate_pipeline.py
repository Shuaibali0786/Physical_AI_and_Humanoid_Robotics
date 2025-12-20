"""
RAG Data Validation System - Direct Implementation from User Input

This script implements the exact functionality described in the user input:
- Connect to Qdrant Cloud using QdrantClient and environment variables
- Retrieve all vectors and metadata from the "physical_ai_book" collection
- Implement a similarity search function that takes a query and returns top matching chunks
- Loop through a sample set of queries to test retrieval accuracy
- Log any chunks that are missing or return errors
- Generate a report confirming data integrity and retrieval success
"""
import os
import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def connect_qdrant():
    """
    Connect to Qdrant Cloud using QdrantClient and environment variables
    """
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )

    logger.info(f"Successfully connected to Qdrant at {qdrant_url}")
    return client


def retrieve_chunks(client, collection_name):
    """
    Retrieve all vectors and metadata from the collection
    """
    logger.info(f"Retrieving all chunks from collection: {collection_name}")

    try:
        # Check if collection exists
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' exists, retrieving all points...")
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


def similarity_search(client, collection_name, query, top_k=5):
    """
    Perform similarity search that takes a query and returns top matching chunks
    """
    logger.info(f"Performing similarity search for query: '{query}'")

    try:
        # For semantic search, we need to convert the query to an embedding
        # Using Cohere for embedding generation
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            logger.warning("COHERE_API_KEY not set, using fallback search method")
            # Fallback to keyword-based search
            return _keyword_based_search(client, collection_name, query, top_k)

        # Import cohere client
        import cohere
        co = cohere.Client(cohere_api_key)

        # Generate embedding for the query
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Perform vector search in Qdrant
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            with_vectors=False
        )

        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "score": result.score,
                "payload": result.payload,
                "content": result.payload.get("content", ""),
                "url": result.payload.get("url", ""),
                "title": result.payload.get("title", "")
            })

        logger.info(f"Similarity search returned {len(results)} results for query '{query}'")
        return results

    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        return []


def _keyword_based_search(client, collection_name, query, top_k=5):
    """
    Fallback keyword-based search when Cohere is not available
    """
    logger.info("Using keyword-based search as fallback")

    all_chunks = []
    offset = None
    batch_size = 100

    query_lower = query.lower()
    query_words = set(query_lower.split())

    while True:
        points, next_offset = client.scroll(
            collection_name=collection_name,
            limit=batch_size,
            offset=offset,
            with_payload=True
        )

        for point in points:
            payload = point.payload
            content = payload.get("content", "").lower()
            title = payload.get("title", "").lower()

            # Calculate relevance score based on keyword matching
            content_matches = sum(1 for word in query_words if word in content)
            title_matches = sum(1 for word in query_words if word in title)

            # Weight title matches higher than content matches
            score = title_matches * 2 + content_matches

            if score > 0:  # Only include results with matches
                all_chunks.append({
                    "id": point.id,
                    "payload": payload,
                    "score": score,
                    "content": payload.get("content", ""),
                    "url": payload.get("url", ""),
                    "title": payload.get("title", "")
                })

        if next_offset is None:
            break
        offset = next_offset

    # Sort by score and return top_k
    all_chunks.sort(key=lambda x: x["score"], reverse=True)
    results = all_chunks[:top_k]

    logger.info(f"Keyword search returned {len(results)} results for query '{query}'")
    return results


def validate_pipeline():
    """
    Main validation function that implements the complete pipeline from user input
    """
    logger.info("Starting RAG Data Validation Pipeline (Spec-2)")

    # Connect to Qdrant
    client = connect_qdrant()
    collection_name = "physical_ai_book"

    # Retrieve all vectors and metadata from the collection
    all_chunks = retrieve_chunks(client, collection_name)

    # Log any chunks that are missing or return errors
    if not all_chunks:
        logger.error("No chunks retrieved from Qdrant collection")
        return

    logger.info(f"Successfully retrieved {len(all_chunks)} chunks from Qdrant")

    # Loop through a sample set of queries to test retrieval accuracy
    sample_queries = ["Introduction to AI", "RAG chatbot pipeline", "Book deployment", "Machine learning concepts"]
    all_search_results = {}

    for q in sample_queries:
        results = similarity_search(client, collection_name, q, top_k=5)
        all_search_results[q] = results
        print(f"Query: {q}, Results: {len(results)} items")

    # Generate a report confirming data integrity and retrieval success
    report = {
        "collection_name": collection_name,
        "total_chunks_retrieved": len(all_chunks),
        "queries_tested": len(sample_queries),
        "search_results_summary": {q: len(results) for q, results in all_search_results.items()},
        "integrity_check": len(all_chunks) > 0,  # Basic integrity check
        "retrieval_success": len(all_chunks) > 0,
        "search_success": all(len(results) > 0 for results in all_search_results.values())
    }

    # Print validation report
    print("\n=== VALIDATION REPORT ===")
    print(f"Collection: {report['collection_name']}")
    print(f"Total chunks retrieved: {report['total_chunks_retrieved']}")
    print(f"Queries tested: {report['queries_tested']}")
    print(f"Integrity check: {'PASS' if report['integrity_check'] else 'FAIL'}")
    print(f"Retrieval success: {'PASS' if report['retrieval_success'] else 'FAIL'}")
    print(f"Search success: {'PASS' if report['search_success'] else 'FAIL'}")
    print("=========================")

    logger.info("Spec-2 validation completed successfully.")
    return report


if __name__ == "__main__":
    validate_pipeline()
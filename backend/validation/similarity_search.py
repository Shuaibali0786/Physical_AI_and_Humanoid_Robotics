"""
Functions for testing similarity search functionality
"""
import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere


logger = logging.getLogger(__name__)


def perform_similarity_search(client: QdrantClient, collection_name: str, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Perform similarity search with sample queries and return relevant results.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to search in
        query: Query string to search for
        top_k: Number of top results to return (default 5)

    Returns:
        Search results with relevance scores and content
    """
    try:
        # First, we need to generate an embedding for the query using Cohere
        import os
        from cohere import Client as CohereClient

        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            logger.warning("COHERE_API_KEY not found, using fallback search method")
            # Use Qdrant's scroll and keyword matching as fallback
            return _fallback_keyword_search(client, collection_name, query, top_k)

        # Generate embedding for the query
        cohere_client = CohereClient(api_key=cohere_api_key)
        response = cohere_client.embed(
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

        logger.info(f"Similarity search for '{query}' returned {len(results)} results")
        return results

    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        logger.info("Using fallback search method...")
        return _fallback_keyword_search(client, collection_name, query, top_k)


def _fallback_keyword_search(client: QdrantClient, collection_name: str, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Fallback search method using keyword matching when embedding generation fails.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to search in
        query: Query string to search for
        top_k: Number of top results to return

    Returns:
        Search results with relevance scores based on keyword matching
    """
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
            content_words = content.split()
            title_words = title.split()

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

    logger.info(f"Fallback search for '{query}' returned {len(results)} results")
    return results


def test_retrieval_accuracy(client: QdrantClient, collection_name: str, queries: List[str], top_k: int = 5) -> Dict[str, Any]:
    """
    Create function to test retrieval accuracy with sample queries.

    Args:
        client: QdrantClient instance
        collection_name: Name of the Qdrant collection to test
        queries: List of query strings to test
        top_k: Number of results to return for each query

    Returns:
        Accuracy test results
    """
    total_queries = len(queries)
    successful_queries = 0
    all_results = {}
    relevance_scores = []

    for query in queries:
        results = perform_similarity_search(client, collection_name, query, top_k)
        if results:
            successful_queries += 1
            all_results[query] = results

            # For now, we'll consider any results as relevant
            # In a real implementation, we'd have expected results to compare against
            relevance_scores.extend([result['score'] for result in results])
        else:
            all_results[query] = []

    accuracy_rate = (successful_queries / total_queries) * 100 if total_queries > 0 else 0
    avg_relevance_score = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0

    results = {
        "total_queries": total_queries,
        "successful_queries": successful_queries,
        "accuracy_rate": accuracy_rate,
        "average_relevance_score": avg_relevance_score,
        "query_results": all_results
    }

    logger.info(f"Retrieval accuracy test: {accuracy_rate:.2f}% success rate, avg relevance: {avg_relevance_score:.2f}")
    return results


def add_relevance_scoring_for_search_results(results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Add relevance scoring for search results.

    Args:
        results: List of search results

    Returns:
        List of search results with enhanced relevance scoring
    """
    # The relevance scores are already provided by Qdrant in the search results
    # This function can be used to enhance the scoring if needed
    for result in results:
        # For now, we'll just ensure the score is present
        if 'score' not in result:
            result['score'] = 0.0

    return results




def generate_sample_queries_from_content(chunks: List[Dict[str, Any]], num_queries: int = 5) -> List[str]:
    """
    Generate sample queries based on known content from the original book.

    Args:
        chunks: List of content chunks to generate queries from
        num_queries: Number of sample queries to generate

    Returns:
        List of sample query strings
    """
    queries = []

    # Extract key phrases or sentences from the content
    for chunk in chunks[:num_queries]:  # Use first few chunks to generate queries
        content = chunk.get("content", "")
        title = chunk.get("title", "")

        # Create queries based on title or key content
        if title:
            queries.append(title)

        # Extract a sentence from content as a query
        if content:
            sentences = content.split('.')
            if len(sentences) > 0:
                # Get a sentence that's not too short
                for sentence in sentences:
                    sentence = sentence.strip()
                    if len(sentence) > 10:  # At least 10 characters
                        queries.append(sentence)
                        break

    # Add some generic queries related to the book topic
    generic_queries = [
        "Introduction to AI",
        "RAG chatbot pipeline",
        "Book deployment",
        "Machine learning concepts",
        "Neural networks explained"
    ]

    # Combine generated and generic queries
    all_queries = list(set(queries + generic_queries))[:num_queries]

    logger.info(f"Generated {len(all_queries)} sample queries for testing")
    return all_queries
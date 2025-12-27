"""
Search Service for Book Content
Provides search functionality that connects to the RAG system
"""
import os
import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from .main import search_in_qdrant, get_qdrant_client

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SearchService:
    def __init__(self):
        self.qdrant_client = get_qdrant_client()
        self.cohere_client = self._get_cohere_client()
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")

    def _get_cohere_client(self) -> cohere.Client:
        """Create and return a Cohere client instance."""
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable not set")

        return cohere.Client(cohere_api_key)

    def search_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for content in the book using vector similarity.

        Args:
            query: The search query string
            top_k: Number of top results to return

        Returns:
            List of search results with content and metadata
        """
        try:
            # Generate embedding for the query
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]

            # Perform similarity search in Qdrant
            results = search_in_qdrant(
                self.qdrant_client,
                self.collection_name,
                query_embedding,
                top_k
            )

            return results

        except Exception as e:
            logger.error(f"Error performing search: {e}")
            return []

    def keyword_search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Perform keyword-based search in the collection.

        Args:
            query: The search query string
            top_k: Number of top results to return

        Returns:
            List of search results with content and metadata
        """
        try:
            # Use Qdrant's full-text search capabilities
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query,
                limit=top_k,
                with_payload=True,
                with_vectors=False,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content",
                            match=models.MatchText(text=query)
                        )
                    ]
                )
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

            logger.info(f"Found {len(results)} results for keyword search")
            return results

        except Exception as e:
            logger.error(f"Error performing keyword search: {e}")
            return []

    def hybrid_search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining vector similarity and keyword matching.

        Args:
            query: The search query string
            top_k: Number of top results to return

        Returns:
            List of search results with content and metadata
        """
        try:
            # Get vector search results
            vector_results = self.search_content(query, top_k * 2)

            # Get keyword search results
            keyword_results = self.keyword_search(query, top_k * 2)

            # Combine and rank results (simplified approach)
            all_results = vector_results + keyword_results

            # Remove duplicates based on content
            seen_content = set()
            unique_results = []
            for result in all_results:
                content = result.get("content", "")
                if content not in seen_content:
                    seen_content.add(content)
                    unique_results.append(result)

            # Sort by score if available, otherwise by content length
            unique_results.sort(
                key=lambda x: x.get("score", len(x.get("content", ""))),
                reverse=True
            )

            return unique_results[:top_k]

        except Exception as e:
            logger.error(f"Error performing hybrid search: {e}")
            # Fallback to vector search
            return self.search_content(query, top_k)


# Singleton instance
search_service = SearchService()


def search_book_content(query: str, search_type: str = "vector", top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Public function to search book content.

    Args:
        query: The search query string
        search_type: Type of search ("vector", "keyword", "hybrid")
        top_k: Number of top results to return

    Returns:
        List of search results with content and metadata
    """
    if search_type == "vector":
        return search_service.search_content(query, top_k)
    elif search_type == "keyword":
        return search_service.keyword_search(query, top_k)
    elif search_type == "hybrid":
        return search_service.hybrid_search(query, top_k)
    else:
        # Default to vector search
        return search_service.search_content(query, top_k)
"""
RAG Book Content Processing System
Main entry point for the ingestion pipeline
"""
import os
import requests
import time
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import logging
from pathlib import Path
import re
from dataclasses import dataclass

import bs4
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class TextChunk:
    """Represents a chunk of text with metadata"""
    id: str
    content: str
    source_url: str
    source_title: str
    chunk_index: int
    metadata: Dict[str, Any]


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
    for i in range(chunk_size, chunk_size - overlap, -1):
        if i < len(text):
            if text[i] in '.!?':
                break_point = i + 1
                break
            elif text[i] == '\n':
                break_point = i
                break
            elif text[i] == ' ':
                break_point = i

    # If we couldn't find a good break point, just break at chunk_size
    if break_point >= len(text):
        break_point = chunk_size

    first_chunk = text[:break_point]
    remaining_text = text[break_point - overlap:]  # Apply overlap

    chunks = [first_chunk]
    if remaining_text.strip():
        chunks.extend(chunk_text_recursive(remaining_text, chunk_size, overlap))

    return chunks


def chunk_content_properly(extracted_content: List[Dict[str, Any]], chunk_size: int = 1000, overlap: int = 200) -> List[TextChunk]:
    """
    Properly chunk the extracted content with metadata preservation.

    Args:
        extracted_content: List of extracted content dictionaries
        chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks

    Returns:
        List of TextChunk objects with proper metadata
    """
    all_chunks = []
    chunk_id_counter = 0

    for content_item in extracted_content:
        text = content_item.get('content', '')
        source_url = content_item.get('url', '')
        source_title = content_item.get('title', '')

        # Split the content into chunks
        text_chunks = chunk_text_recursive(text, chunk_size, overlap)

        # Create TextChunk objects with metadata
        for i, chunk_text in enumerate(text_chunks):
            chunk = TextChunk(
                id=f"chunk_{chunk_id_counter}",
                content=chunk_text,
                source_url=source_url,
                source_title=source_title,
                chunk_index=i,
                metadata={
                    'original_content_length': len(text),
                    'chunk_size': len(chunk_text),
                    'total_chunks': len(text_chunks),
                    'content_type': content_item.get('content_type', 'text')
                }
            )
            all_chunks.append(chunk)
            chunk_id_counter += 1

    return all_chunks


def clean_text(text: str) -> str:
    """
    Clean and preprocess text content.

    Args:
        text: Raw text to clean

    Returns:
        Cleaned text
    """
    if not text:
        return ""

    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)

    # Remove leading/trailing whitespace
    text = text.strip()

    # Normalize quotes and other special characters if needed
    # text = text.replace('“', '"').replace('”', '"').replace('‘', "'").replace('’', "'")

    return text


def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance.

    Returns:
        QdrantClient instance
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )

    return client


def get_cohere_client() -> cohere.Client:
    """
    Create and return a Cohere client instance.

    Returns:
        Cohere Client instance
    """
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    return cohere.Client(cohere_api_key)


def get_all_urls(base_url: str = None) -> List[str]:
    """
    Discover all book page URLs from the deployed site.

    Args:
        base_url: The base URL of the deployed book site

    Returns:
        List of URLs to crawl
    """
    if base_url is None:
        base_url = os.getenv("BOOK_SITE_URL", "https://vercel.com/shuaib-alis-projects-3de375c3/ai-physical-book")

    logger.info(f"Discovering URLs from: {base_url}")

    # First try to get URLs from sitemap
    sitemap_url = urljoin(base_url, "sitemap.xml")
    urls = []

    try:
        response = requests.get(sitemap_url)
        if response.status_code == 200:
            soup = BeautifulSoup(response.content, 'xml')
            for loc in soup.find_all('loc'):
                url = loc.text.strip()
                if url.startswith(base_url):
                    urls.append(url)
        else:
            logger.warning(f"Sitemap not found at {sitemap_url}, using base URL only")
            urls = [base_url]
    except Exception as e:
        logger.error(f"Error fetching sitemap: {e}")
        # Fallback to just the base URL
        urls = [base_url]

    logger.info(f"Discovered {len(urls)} URLs to process")
    return urls


def extract_text_from_url(url: str) -> Optional[Dict[str, Any]]:
    """
    Extract clean text content from a given URL.

    Args:
        url: The URL to extract content from

    Returns:
        Dictionary with url, title, and content, or None if extraction fails
    """
    logger.info(f"Extracting text from: {url}")

    try:
        # Add headers to mimic a real browser request
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        }

        response = requests.get(url, headers=headers)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to find the main content area (specific to Docusaurus)
        # Common Docusaurus selectors for main content
        content_selectors = [
            'article',
            '[class*="docItemContainer"]',
            '[class*="markdown"]',
            '[class*="container"]',
            'main',
            '[role="main"]',
            '.main-wrapper',
            '.theme-doc-markdown',
            '[class*="docContent"]',
            '[class*="theme-doc"]'
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        if not content_element:
            # If no specific content area found, use body
            content_element = soup.find('body')

        if content_element:
            # Get text content, removing extra whitespace
            text_content = content_element.get_text(separator=' ', strip=True)

            # Clean up excessive whitespace
            text_content = re.sub(r'\s+', ' ', text_content)

            # Get page title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else "Untitled"

            # Detect content type (text, code, etc.)
            content_type = "text"
            if soup.find('code') or soup.find('pre'):
                content_type = "text_with_code"

            return {
                'url': url,
                'title': title,
                'content': clean_text(text_content),
                'content_type': content_type
            }
        else:
            logger.warning(f"No content found in {url}")
            return None

    except requests.exceptions.RequestException as e:
        logger.error(f"Network error extracting text from {url}: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error extracting text from {url}: {e}")
        return None


def validate_and_filter_urls(urls: List[str], base_url: str) -> List[str]:
    """
    Validate and filter URLs to ensure they're valid and from the same domain.

    Args:
        urls: List of URLs to validate
        base_url: Base URL for domain validation

    Returns:
        List of validated and filtered URLs
    """
    from urllib.parse import urlparse

    base_domain = urlparse(base_url).netloc
    valid_urls = []

    for url in urls:
        try:
            parsed = urlparse(url)
            if parsed.netloc == base_domain:
                valid_urls.append(url)
            else:
                logger.debug(f"Skipping external URL: {url}")
        except Exception:
            logger.warning(f"Invalid URL format: {url}")

    return valid_urls


def create_embedding(text_chunks: List[str]) -> List[List[float]]:
    """
    Generate vector embeddings for text chunks using Cohere.

    Args:
        text_chunks: List of text chunks to generate embeddings for

    Returns:
        List of embeddings (each embedding is a list of floats)
    """
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    co = cohere.Client(cohere_api_key)

    # Cohere has limits on batch size, so we'll process in chunks
    batch_size = 96  # Cohere's limit is 96 texts per request
    all_embeddings = []

    for i in range(0, len(text_chunks), batch_size):
        batch = text_chunks[i:i + batch_size]
        logger.info(f"Generating embeddings for batch {i//batch_size + 1}/{(len(text_chunks)-1)//batch_size + 1}")

        # Add retry mechanism for Cohere API calls
        max_retries = 3
        retry_count = 0
        success = False

        while retry_count < max_retries and not success:
            try:
                response = co.embed(
                    texts=batch,
                    model="embed-english-v3.0",  # Using a more recent Cohere model
                    input_type="search_document"
                )

                embeddings = response.embeddings
                all_embeddings.extend(embeddings)
                success = True

            except cohere.CohereAPIError as e:
                retry_count += 1
                logger.warning(f"Cohere API error (attempt {retry_count}/{max_retries}): {e}")
                if retry_count < max_retries:
                    time.sleep(2 ** retry_count)  # Exponential backoff
                else:
                    logger.error(f"Failed to generate embeddings after {max_retries} attempts")
                    # Return empty embeddings for failed batch to maintain order
                    all_embeddings.extend([[] for _ in range(len(batch))])

            except Exception as e:
                logger.error(f"Unexpected error generating embeddings: {e}")
                # Return empty embeddings for failed batch to maintain order
                all_embeddings.extend([[] for _ in range(len(batch))])
                break

    return all_embeddings


def create_embedding_with_caching(text_chunks: List[str], cache_file: str = "embedding_cache.json") -> List[List[float]]:
    """
    Generate vector embeddings with caching to avoid redundant API calls.

    Args:
        text_chunks: List of text chunks to generate embeddings for
        cache_file: File to store cached embeddings

    Returns:
        List of embeddings (each embedding is a list of floats)
    """
    import hashlib
    import json

    # Load existing cache if it exists
    cache = {}
    try:
        with open(cache_file, 'r', encoding='utf-8') as f:
            cache = json.load(f)
    except FileNotFoundError:
        logger.info(f"Cache file {cache_file} not found, starting fresh cache")

    # Create embeddings for chunks not in cache
    embeddings = []
    for text in text_chunks:
        # Create a hash of the text to use as cache key
        text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()

        if text_hash in cache:
            # Use cached embedding
            embeddings.append(cache[text_hash])
            logger.debug(f"Used cached embedding for text: {text[:50]}...")
        else:
            # Generate new embedding and cache it
            new_embeddings = create_embedding([text])
            if new_embeddings and len(new_embeddings) > 0 and len(new_embeddings[0]) > 0:
                embeddings.append(new_embeddings[0])
                cache[text_hash] = new_embeddings[0]

                # Save cache periodically
                with open(cache_file, 'w', encoding='utf-8') as f:
                    json.dump(cache, f)
            else:
                # If embedding generation failed, add an empty list
                embeddings.append([])
                logger.warning(f"Failed to generate embedding for text: {text[:50]}...")

    return embeddings


def create_collection(collection_name: str = "physical_ai_book") -> QdrantClient:
    """
    Create a collection in Qdrant Cloud for storing vectors.

    Args:
        collection_name: Name of the collection to create

    Returns:
        QdrantClient instance
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    # Initialize Qdrant client
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )

    # Check if collection exists
    try:
        client.get_collection(collection_name)
        logger.info(f"Collection '{collection_name}' already exists, recreating...")
        client.delete_collection(collection_name)
    except:
        logger.info(f"Collection '{collection_name}' does not exist, creating...")

    # Create collection with appropriate vector size (Cohere embeddings are 1024-dim by default)
    client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )

    logger.info(f"Collection '{collection_name}' created successfully")
    return client


def save_chunk_to_qdrant(client: QdrantClient, collection_name: str, chunks: List[TextChunk], embeddings: List[List[float]]):
    """
    Save text chunks with their embeddings to Qdrant Cloud.

    Args:
        client: QdrantClient instance
        collection_name: Name of the collection to save to
        chunks: List of TextChunk objects with content and metadata
        embeddings: List of embeddings corresponding to the chunks
    """
    if len(chunks) != len(embeddings):
        raise ValueError(f"Number of chunks ({len(chunks)}) must match number of embeddings ({len(embeddings)})")

    # Prepare points for insertion
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        if not embedding:  # Skip chunks with empty embeddings
            continue

        point = models.PointStruct(
            id=i,
            vector=embedding,
            payload={
                "url": chunk.source_url,
                "title": chunk.source_title,
                "content": chunk.content,
                "chunk_index": chunk.chunk_index,
                "content_type": chunk.metadata.get('content_type', 'text'),
                "original_content_length": chunk.metadata.get('original_content_length', 0),
                "chunk_size": chunk.metadata.get('chunk_size', 0),
                "total_chunks": chunk.metadata.get('total_chunks', 0),
                "created_at": time.time()
            }
        )
        points.append(point)

    if points:
        logger.info(f"Saving {len(points)} chunks to Qdrant collection '{collection_name}'")
        client.upsert(
            collection_name=collection_name,
            points=points
        )
        logger.info(f"Successfully saved {len(points)} chunks to Qdrant")
    else:
        logger.warning("No valid chunks to save to Qdrant")


def search_in_qdrant(client: QdrantClient, collection_name: str, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Perform similarity search in Qdrant collection.

    Args:
        client: QdrantClient instance
        collection_name: Name of the collection to search in
        query_embedding: Embedding vector to search for similar items
        top_k: Number of top results to return

    Returns:
        List of search results with content and metadata
    """
    try:
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k
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

        logger.info(f"Found {len(results)} results for similarity search")
        return results

    except Exception as e:
        logger.error(f"Error performing similarity search: {e}")
        return []


def main():
    """
    Main function to execute the complete pipeline:
    1. Discover URLs
    2. Extract text from each URL
    3. Chunk content properly
    4. Generate embeddings
    5. Save to Qdrant
    """
    start_time = time.time()
    logger.info("Starting RAG Book Content Processing Pipeline")

    try:
        # Step 1: Get all URLs
        urls = get_all_urls()

        # Validate and filter URLs
        base_url = os.getenv("BOOK_SITE_URL", "https://vercel.com/shuaib-alis-projects-3de375c3/ai-physical-book")
        urls = validate_and_filter_urls(urls, base_url)

        # Step 2: Extract text from each URL
        extracted_contents = []
        failed_urls = []
        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
            content = extract_text_from_url(url)
            if content:
                extracted_contents.append(content)
            else:
                failed_urls.append(url)
                logger.warning(f"Failed to extract content from {url}")

            # Be respectful to the server
            time.sleep(1)

        if not extracted_contents:
            logger.error("No content extracted from any URLs")
            return

        # Step 3: Properly chunk the content
        chunk_size = int(os.getenv("CHUNK_SIZE", "1000"))
        chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "200"))
        text_chunks = chunk_content_properly(extracted_contents, chunk_size, chunk_overlap)

        logger.info(f"Created {len(text_chunks)} text chunks from {len(extracted_contents)} content items")

        # Step 4: Prepare text contents for embedding
        text_contents = [chunk.content for chunk in text_chunks]

        # Step 5: Generate embeddings
        logger.info("Generating embeddings for text chunks...")
        embeddings = create_embedding(text_contents)

        # Step 6: Create Qdrant collection and save chunks
        collection_name = "physical_ai_book"
        logger.info(f"Creating Qdrant collection: {collection_name}")
        client = create_collection(collection_name)

        # Save chunks to Qdrant
        logger.info("Saving chunks to Qdrant...")
        save_chunk_to_qdrant(client, collection_name, text_chunks, embeddings)

        # Calculate and report metrics
        total_time = time.time() - start_time
        total_content_chars = sum(len(content.get('content', '')) for content in extracted_contents)
        logger.info(f"Pipeline completed successfully in {total_time:.2f} seconds")
        logger.info(f"Processed {len(urls)} URLs ({len(failed_urls)} failed)")
        logger.info(f"Extracted {len(extracted_contents)} content items ({total_content_chars} characters total)")
        logger.info(f"Created {len(text_chunks)} text chunks")
        logger.info(f"Generated {len(embeddings)} embeddings")
        logger.info(f"Saved to Qdrant collection: {collection_name}")

    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        raise


if __name__ == "__main__":
    main()
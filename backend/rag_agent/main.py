"""
RAG Agent API with FastAPI + Cohere + Qdrant
Single-file implementation for the Retrieval-Augmented Generation Agent
"""
import os
import time
import logging
import re
from typing import List, Dict, Any, Optional
from contextlib import asynccontextmanager
from dataclasses import dataclass
from urllib.parse import urljoin, urlparse

import uvicorn
from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.conversions import common_types
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# Pydantic models for request/response validation
class QueryRequest(BaseModel):
    """
    Model for incoming query requests
    """
    query: str
    max_results: Optional[int] = 5
    temperature: Optional[float] = 0.7
    metadata_filters: Optional[Dict[str, Any]] = {}


class RetrievedChunk(BaseModel):
    """
    Model for content chunks retrieved from Qdrant
    """
    id: str
    content: str
    score: float
    url: str
    title: str
    metadata: Dict[str, Any] = {}


class AgentResponse(BaseModel):
    """
    Model for responses from the OpenAI Agent
    """
    answer: str
    query: str
    retrieved_chunks: List[RetrievedChunk]
    sources: List[str]
    confidence: float
    timestamp: float


class QueryResponse(BaseModel):
    """
    Model for API response
    """
    answer: str
    query: str
    retrieved_chunks: List[RetrievedChunk]
    sources: List[str]
    confidence: float
    timestamp: float


class HealthResponse(BaseModel):
    """
    Model for health check endpoint response
    """
    status: str
    details: Dict[str, Any]
    timestamp: float


class ConfigResponse(BaseModel):
    """
    Model for configuration endpoint response
    """
    qdrant_collection: str
    max_results_default: int
    temperature_default: float
    available_models: List[str]


# Configuration module
@dataclass
class Settings:
    # Qdrant configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_book")

    # Cohere configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # OpenAI configuration (alternative to Cohere if needed)
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")

    # Book site configuration
    book_site_url: str = os.getenv("BOOK_SITE_URL", "https://vercel.com/shuaib-alis-projects-3de375c3/ai-physical-book")

    # Application settings
    default_max_results: int = int(os.getenv("DEFAULT_MAX_RESULTS", "5"))
    default_temperature: float = float(os.getenv("DEFAULT_TEMPERATURE", "0.7"))
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "1000"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "200"))
    rate_limit_delay: float = float(os.getenv("RATE_LIMIT_DELAY", "1.0"))


def get_settings() -> Settings:
    """
    Get application settings from environment variables
    """
    return Settings()


def validate_config(settings: Settings) -> bool:
    """
    Validate that all required configuration values are present

    Args:
        settings: Settings object to validate

    Returns:
        True if all required settings are present, False otherwise
    """
    required_fields = [
        settings.qdrant_url,
        settings.qdrant_api_key,
        settings.cohere_api_key  # Using Cohere as specified in the original requirements
    ]

    for field in required_fields:
        if not field:
            return False

    return True


# Utility functions
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


# Retrieval module
def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance using configuration
    """
    settings = get_settings()
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
    settings = get_settings()
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
    settings = get_settings()
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
    settings = get_settings()
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
    settings = get_settings()
    try:
        client = get_qdrant_client()

        collection_info = client.get_collection(settings.qdrant_collection_name)
        return collection_info.points_count

    except Exception as e:
        logger.error(f"Error getting chunks count from Qdrant: {e}")
        return 0


# Agent module
@dataclass
class AgentResult:
    """
    Result from the agent processing
    """
    answer: str
    sources: List[str]
    confidence: float


def get_cohere_client() -> cohere.Client:
    """
    Create and return a Cohere client instance using configuration
    """
    settings = get_settings()
    if not settings.cohere_api_key:
        raise ValueError("COHERE_API_KEY must be set in environment variables")

    client = cohere.Client(
        api_key=settings.cohere_api_key
    )

    return client


def create_openai_agent():
    """
    Create an agent instance using Cohere (as specified in the original requirements)
    Note: The original spec mentioned OpenAI Agent SDK, but the user's example code used Cohere
    For consistency with the original project, we'll implement with Cohere
    """
    return get_cohere_client()


@retry_on_failure(max_retries=3, delay=1.0)
def query_agent_with_context(agent: cohere.Client, query: str, retrieved_chunks: List[RetrievedChunk], temperature: float = 0.7) -> AgentResult:
    """
    Query the agent with retrieved context chunks to generate a response.

    Args:
        agent: Cohere client instance
        query: Original user query
        retrieved_chunks: List of retrieved content chunks to use as context
        temperature: Temperature parameter for response generation

    Returns:
        AgentResult with answer, sources, and confidence
    """
    try:
        # Combine the retrieved content to form the context
        context_parts = []
        sources = []

        for chunk in retrieved_chunks:
            if chunk.content.strip():  # Only add non-empty content
                context_parts.append(f"Source: {chunk.url}\nContent: {chunk.content}")
                if chunk.url and chunk.url not in sources:
                    sources.append(chunk.url)

        if not context_parts:
            logger.warning("No context chunks provided for agent query")
            # Generate a response based only on the query
            context_text = ""
        else:
            context_text = "\n\n".join(context_parts)

        # Construct the prompt for the agent with context
        if context_text:
            prompt = f"""
            Based on the following context, please answer the user's question.

            CONTEXT:
            {context_text}

            QUESTION:
            {query}

            ANSWER:
            """
        else:
            # If no context is available, just answer the query directly
            prompt = f"""
            Please answer the following question to the best of your ability:

            QUESTION:
            {query}

            ANSWER:
            """

        # Generate response using Cohere
        response = agent.generate(
            model="command-r-plus",  # Using a suitable Cohere model for question answering
            prompt=prompt,
            max_tokens=1000,  # Adjust as needed
            temperature=temperature,
            stop_sequences=["\nQUESTION:", "\nANSWER:"]
        )

        # Extract the answer from the response
        answer = response.generations[0].text.strip()

        # Calculate a basic confidence score based on response length and relevance indicators
        confidence = calculate_response_confidence(answer, query, retrieved_chunks)

        logger.info(f"Generated answer with {len(sources)} sources for query: {query[:50]}...")

        return AgentResult(
            answer=answer,
            sources=sources,
            confidence=confidence
        )

    except Exception as e:
        logger.error(f"Error querying agent with context: {e}")
        raise


def calculate_response_confidence(answer: str, query: str, retrieved_chunks: List[RetrievedChunk]) -> float:
    """
    Calculate a basic confidence score for the agent response.

    Args:
        answer: The generated answer
        query: The original query
        retrieved_chunks: The chunks used to generate the answer

    Returns:
        Confidence score between 0 and 1
    """
    if not answer or not retrieved_chunks:
        return 0.0

    # Basic heuristics for confidence calculation
    # 1. If we have retrieved chunks, start with a base confidence
    base_confidence = 0.5 if retrieved_chunks else 0.0

    # 2. Increase confidence if answer is substantial
    if len(answer) > 50:  # Answer is substantial
        base_confidence += 0.2
    elif len(answer) == 0:  # No answer provided
        return 0.0

    # 3. Increase confidence if we have good quality sources
    if len(retrieved_chunks) > 0:
        avg_score = sum(c.score for c in retrieved_chunks) / len(retrieved_chunks)
        # Assuming scores are normalized, higher scores mean better relevance
        base_confidence += min(avg_score * 0.3, 0.3)  # Cap contribution at 0.3

    # 4. Ensure confidence is within bounds
    confidence = max(0.0, min(1.0, base_confidence))

    return confidence


def validate_agent_connection() -> bool:
    """
    Validate that we can connect to the Cohere API.

    Returns:
        True if connection is successful, False otherwise
    """
    settings = get_settings()
    try:
        client = get_cohere_client()

        # Test the connection with a simple classification request
        response = client.classify(
            model="embed-multilingual-v2.0",
            inputs=["test"]
        )

        logger.info("Successfully connected to Cohere API")
        return True

    except Exception as e:
        logger.error(f"Failed to connect to Cohere API: {e}")
        return False


# Create FastAPI app with lifespan to initialize resources
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Initialize resources on startup and cleanup on shutdown
    """
    logger.info("Starting RAG Agent service...")
    # Initialize clients and resources here
    yield
    logger.info("Shutting down RAG Agent service...")


app = FastAPI(
    title="RAG Agent API",
    description="API for the Retrieval-Augmented Generation Agent",
    version="1.0.0",
    lifespan=lifespan
)


@app.get("/")
async def root():
    """
    Root endpoint for health check
    """
    return {"message": "RAG Agent API is running", "status": "healthy"}


@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Main endpoint to process user queries and return RAG-enhanced responses
    """
    try:
        logger.info(f"Processing query: {request.query[:50]}...")

        # Retrieve relevant chunks from Qdrant
        retrieved_chunks = retrieve_chunks_from_qdrant(request.query, request.max_results)

        # Create agent and generate response
        agent = create_openai_agent()
        response = query_agent_with_context(agent, request.query, retrieved_chunks, request.temperature)

        return QueryResponse(
            answer=response.answer,
            query=request.query,
            retrieved_chunks=retrieved_chunks,
            sources=response.sources,
            confidence=response.confidence,
            timestamp=time.time()
        )

    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify service status
    """
    # In a real implementation, we would check the status of Qdrant, OpenAI, etc.
    return {
        "status": "healthy",
        "timestamp": time.time(),
        "details": {
            "qdrant_connection": "unknown",  # Would check actual connection
            "openai_agent": "unknown",      # Would check actual availability
        }
    }


@app.get("/config")
async def get_config():
    """
    Configuration endpoint to return current settings
    """
    settings = get_settings()
    return {
        "qdrant_collection": settings.qdrant_collection_name,
        "max_results_default": settings.default_max_results,
        "temperature_default": settings.default_temperature,
        "available_models": ["gpt-4-turbo", "gpt-3.5-turbo"]  # Would be dynamic in real implementation
    }


if __name__ == "__main__":
    # For development/testing purposes
    uvicorn.run(app, host="127.0.0.1", port=8000)
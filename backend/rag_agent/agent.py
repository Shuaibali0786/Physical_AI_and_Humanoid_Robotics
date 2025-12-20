"""
Module for OpenAI Agent functionality
"""
import os
import logging
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import cohere

from .config import get_settings
from .models import RetrievedChunk, AgentResponse
from .utils import retry_on_failure


logger = logging.getLogger(__name__)
settings = get_settings()


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
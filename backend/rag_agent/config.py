"""
Configuration module for RAG Agent
Handles environment variables and application settings
"""
import os
from typing import Optional
from dataclasses import dataclass


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
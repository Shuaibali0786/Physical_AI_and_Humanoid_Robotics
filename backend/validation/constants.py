"""
Constants and configuration parameters for the RAG Data Validation System
"""
from pathlib import Path
import os

# Default configuration
DEFAULT_QDRANT_HOST = "localhost"
DEFAULT_QDRANT_PORT = 6333
DEFAULT_COLLECTION_NAME = "physical_ai_book"
DEFAULT_EMBEDDING_DIMENSION = 1024  # For Cohere embeddings
DEFAULT_TOP_K_RESULTS = 5
DEFAULT_TIMEOUT = 30  # seconds

# Validation thresholds
EMBEDDING_VALIDATION_THRESHOLD = 0.95  # 95% of embeddings should be valid
SEARCH_ACCURACY_THRESHOLD = 0.90  # 90% search accuracy required

# File paths
PROJECT_ROOT = Path(__file__).parent.parent
ENV_FILE_PATH = PROJECT_ROOT / ".env"
DEFAULT_REPORT_PATH = PROJECT_ROOT / "validation_report.json"

# Environment variable names
QDRANT_URL_ENV = "QDRANT_URL"
QDRANT_API_KEY_ENV = "QDRANT_API_KEY"
QDRANT_HOST_ENV = "QDRANT_HOST"
QDRANT_PORT_ENV = "QDRANT_PORT"
COHERE_API_KEY_ENV = "COHERE_API_KEY"

# Validation parameters
VALIDATION_BATCH_SIZE = 100
MAX_CONTENT_LENGTH = 10000  # Maximum content length to process
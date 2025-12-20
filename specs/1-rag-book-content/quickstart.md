# Quickstart: RAG Book Content Processing System

## Prerequisites
- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant Cloud API key and URL

## Setup
1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies with `uv sync` or `uv pip install`
4. Create a `.env` file with required environment variables

## Environment Variables
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
BOOK_SITE_URL=https://vercel.com/shuaib-alis-projects-3de375c3/ai-physical-book
```

## Running the Pipeline
1. Execute the main script: `python main.py`
2. The script will:
   - Discover all book URLs
   - Extract text content from each URL
   - Generate embeddings for content chunks
   - Create the physical_ai_book collection in Qdrant
   - Store all chunks with embeddings in Qdrant

## Expected Output
- All book content processed and stored in Qdrant
- Console output showing progress and statistics
- Error handling for failed URLs or API issues
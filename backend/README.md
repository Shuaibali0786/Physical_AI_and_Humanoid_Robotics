# RAG Book Content Processing System

This project implements a system to crawl, extract, embed, and store book content in a vector database for use in a Retrieval-Augmented Generation (RAG) system.

## Overview

The system performs the following tasks:
1. Discovers all pages from a deployed book site
2. Crawls and extracts clean text content from each page
3. Chunks the content appropriately while preserving context
4. Generates vector embeddings using Cohere
5. Stores the embeddings in Qdrant Cloud with metadata

## Prerequisites

- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant Cloud account and API credentials

## Setup

1. Clone the repository
2. Navigate to the backend directory: `cd backend`
3. Install dependencies using uv:
   ```bash
   uv sync  # or use pip install -r requirements.txt
   ```
4. Copy the `.env` file and add your API keys:
   ```bash
   cp .env .env.local
   # Edit .env.local and add your API keys
   ```

## Configuration

Create a `.env` file (or edit `.env.local`) with the following variables:

```env
# Cohere API configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud configuration
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here

# Book site configuration
BOOK_SITE_URL=https://vercel.com/shuaib-alis-projects-3de375c3/ai-physical-book

# Application settings
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
```

## Usage

Run the main pipeline:

```bash
python main.py
```

This will:
1. Discover all book URLs from the sitemap
2. Extract content from each page
3. Chunk the content appropriately
4. Generate embeddings using Cohere
5. Store everything in Qdrant Cloud

## Architecture

The system is implemented in a single file (`main.py`) with the following key functions:

- `get_all_urls()`: Discovers book page URLs
- `extract_text_from_url()`: Extracts clean text content from a URL
- `chunk_content_properly()`: Splits content into appropriately sized chunks
- `create_embedding()`: Generates vector embeddings using Cohere
- `create_collection()`: Sets up Qdrant collection
- `save_chunk_to_qdrant()`: Stores embeddings in Qdrant
- `main()`: Orchestrates the complete pipeline

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `BOOK_SITE_URL`: The base URL of the book site to process
- `CHUNK_SIZE`: Maximum size of text chunks (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 200)

## Error Handling

The system includes:
- Network error handling with retries
- Cohere API error handling with exponential backoff
- Graceful handling of failed URL extractions
- Validation of inputs and outputs

## Output

The system will create a Qdrant collection named `physical_ai_book` containing the vector embeddings of the book content with associated metadata.

## Performance

The pipeline includes metrics reporting to track:
- Processing time
- Number of URLs processed
- Number of content items extracted
- Number of text chunks created
- Number of embeddings generated
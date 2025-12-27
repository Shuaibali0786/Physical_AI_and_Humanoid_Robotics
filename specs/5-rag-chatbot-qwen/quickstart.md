# Quickstart Guide: RAG Chatbot with Qwen Integration

**Feature**: RAG Chatbot with Qwen Integration
**Spec**: specs/5-rag-chatbot-qwen/spec.md
**Plan**: specs/5-rag-chatbot-qwen/plan.md
**Tasks**: specs/5-rag-chatbot-qwen/tasks.md
**Created**: 2025-12-26

## Overview
This guide provides quick setup instructions for the Qwen-based RAG chatbot that integrates with the existing system using Cohere embeddings and Docusaurus frontend.

## Prerequisites
- Python 3.8+
- Access to Qwen API (API key)
- Existing Qdrant database with Cohere embeddings
- Environment variables configured for Qwen API

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
cd your-project-directory
```

### 2. Install Dependencies
```bash
pip install -r requirements-qwen.txt  # or add Qwen dependencies to existing requirements
```

### 3. Configure Environment Variables
Create or update `.env` file with:
```
QWEN_API_KEY=your_qwen_api_key
QWEN_MODEL=qwen-plus  # or your preferred Qwen model
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key  # if needed for compatibility
```

### 4. Run the Qwen RAG Service
```bash
python -m src.qwen_rag.main
```

## Testing
1. Start the service: `python -m src.qwen_rag.main`
2. Test the API endpoint: `curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d '{"query": "your question here"}'`
3. Verify the response contains Qwen-generated content based on retrieved context

## Key Endpoints
- `POST /query` - Submit a query to the Qwen RAG system
- `GET /health` - Check service health status
- `GET /docs` - Interactive API documentation

## Troubleshooting
- If Qwen API returns errors, verify your API key and quota
- If retrieval fails, check Qdrant connection and existing embeddings
- If responses are not context-aware, verify the RAG pipeline is properly connecting retrieval to generation
# Quickstart: RAG Agent with OpenAI Agent SDK + FastAPI

## Prerequisites
- Python 3.11+
- FastAPI
- OpenAI API key
- Qdrant Cloud account and API credentials
- Access to the "physical_ai_book" collection in Qdrant

## Setup
1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies with `pip install -r requirements-agent.txt`
4. Create a `.env` file with required environment variables

## Environment Variables
```
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
OPENAI_API_KEY=your_openai_api_key
QDRANT_HOST=your_qdrant_host (if applicable)
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=physical_ai_book
```

## Running the Agent
1. Start the FastAPI server: `uvicorn backend.rag_agent.main:app --reload --port 8000`
2. The API will be available at `http://localhost:8000`

## Using the API
1. Send a POST request to `http://localhost:8000/query` with a JSON body containing your query:
   ```
   {
     "query": "What is the main concept discussed in chapter 1?",
     "max_results": 5,
     "temperature": 0.7
   }
   ```
2. Receive a response with the answer and retrieved context

## Testing
- Run the test suite: `pytest tests/test_agent.py`
- Test endpoints: `pytest tests/test_endpoints.py`
- Test retrieval logic: `pytest tests/test_retrieval.py`
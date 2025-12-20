# API Contract: RAG Agent Frontend Integration

## Overview
This document defines the API contract between the Docusaurus frontend and the FastAPI RAG agent backend.

## Base URL
The backend API is expected to be available at a configurable base URL (e.g., `http://localhost:8000` for local development or a deployed URL).

## Endpoints

### POST /query
Process user queries and return RAG-enhanced responses.

#### Request
- **Method**: POST
- **Path**: `/query`
- **Content-Type**: `application/json`

**Request Body**:
```json
{
  "query": "string (required) - The user's question",
  "max_results": "integer (optional, default: 5) - Number of results to retrieve",
  "temperature": "number (optional, default: 0.7) - Temperature for response generation",
  "metadata_filters": "object (optional, default: {}) - Metadata filters for retrieval"
}
```

**Example Request**:
```json
{
  "query": "What are the key principles of physical AI?",
  "max_results": 5,
  "temperature": 0.7,
  "metadata_filters": {}
}
```

#### Response
**Success Response (200 OK)**:
```json
{
  "answer": "string - The generated answer",
  "query": "string - The original query",
  "retrieved_chunks": [
    {
      "id": "string - Chunk identifier",
      "content": "string - The actual content text",
      "score": "number - Relevance score",
      "url": "string - Source URL",
      "title": "string - Content title",
      "metadata": "object - Additional metadata"
    }
  ],
  "sources": ["string - Array of source URLs"],
  "confidence": "number - Confidence score (0-1)",
  "timestamp": "number - Unix timestamp"
}
```

**Error Response (500 Internal Server Error)**:
```json
{
  "detail": "string - Error message"
}
```

### GET /health
Health check endpoint to verify service status.

#### Request
- **Method**: GET
- **Path**: `/health`

#### Response
**Success Response (200 OK)**:
```json
{
  "status": "string - Health status",
  "timestamp": "number - Unix timestamp",
  "details": {
    "qdrant_connection": "string - Qdrant connection status",
    "openai_agent": "string - Agent availability status"
  }
}
```

### GET /config
Configuration endpoint to return current settings.

#### Request
- **Method**: GET
- **Path**: `/config`

#### Response
**Success Response (200 OK)**:
```json
{
  "qdrant_collection": "string - Qdrant collection name",
  "max_results_default": "number - Default max results",
  "temperature_default": "number - Default temperature",
  "available_models": ["string - Array of available model names"]
}
```

## Frontend Implementation Contract

### Client-Side Requirements
The frontend implementation must handle:

1. **Request Interception**: Capture user queries from the chat interface
2. **Context Enhancement**: Add page context information to queries when available
3. **Loading States**: Display appropriate loading indicators during API calls
4. **Error Handling**: Gracefully handle API errors and network failures
5. **Response Processing**: Parse and display agent responses appropriately
6. **Rate Limiting**: Implement client-side rate limiting to prevent API abuse

### Frontend-to-Backend Data Flow
1. User submits query in chat interface
2. Frontend captures query and current page context
3. Frontend makes POST request to `/query` endpoint
4. Backend processes query with RAG agent
5. Backend returns response with answer and metadata
6. Frontend displays response in chat interface

### Expected Response Time
- 95% of queries should return within 10 seconds
- Loading indicators should be shown immediately upon query submission

## Security Considerations
- Input validation must be performed on all user queries to prevent injection attacks
- API requests should include appropriate headers for security
- Client-side rate limiting should be implemented to prevent API abuse
# Data Model: RAG Chatbot with Qwen Integration

## Overview
This document describes the data model for the Qwen-based RAG chatbot that integrates with the existing system using Cohere embeddings and Docusaurus frontend.

## Data Flow
1. User Query → 2. Qwen RAG Agent → 3. Context Retrieval → 4. Qwen Generation → 5. Response

## Key Data Structures

### Frontend UI Configuration
```
{
  "ui_elements": {
    "chat_button": {
      "type": "floating_action_button",
      "icon": "chat_bubble",    // Distinctive chat/messaging icon
      "position": "bottom_right", // Always visible position
      "visible": "boolean"      // Whether the button is currently visible
    },
    "chat_interface": {
      "type": "sidebar_panel",  // Or slide-out panel
      "position": "right",      // Sidebar position
      "width": "string",        // Panel width (e.g., "350px" or "30%")
      "animation": "slide_in"   // Animation type when opening/closing
    }
  }
}
```

### Query Request
```
{
  "query": "string",           // The user's question or query
  "context": "string",         // Optional context (e.g., current page URL or title)
  "session_id": "string",      // Optional session identifier for conversation history
  "metadata": "object"         // Optional additional metadata
}
```

### Retrieved Context
```
{
  "chunks": [
    {
      "id": "string",          // Unique identifier for the chunk
      "content": "string",     // The text content of the chunk
      "url": "string",         // Source URL of the content
      "title": "string",       // Title of the source page
      "score": "number",       // Relevance score from similarity search
      "metadata": "object"     // Additional metadata
    }
  ],
  "query_embedding": "array"   // Embedding vector of the original query
}
```

### Qwen Response
```
{
  "response": "string",        // The generated response from Qwen
  "sources": [                // List of sources used in the response
    {
      "url": "string",         // Source URL
      "title": "string",       // Source title
      "content_snippet": "string" // Relevant snippet
    }
  ],
  "confidence": "number",      // Confidence score for the response
  "query": "string",          // Echo of the original query
  "timestamp": "string"       // When the response was generated
}
```

### API Response Format
```
{
  "status": "string",          // "success" or "error"
  "data": {                    // Response data object
    "response": "string",      // The answer from Qwen
    "sources": [              // Sources used in the response
      {
        "url": "string",
        "title": "string",
        "content_snippet": "string"
      }
    ],
    "query": "string"         // Echo of the original query
  },
  "error": "string"           // Error message if status is "error"
}
```

## Integration with Existing Data
- Uses existing Qdrant collection with Cohere embeddings
- Maintains same metadata structure as previous specs
- Compatible with existing frontend data expectations

## Schema Validation
- Query must be non-empty string
- Context and session_id are optional
- Response must be non-empty string
- Sources array may be empty if no relevant context found
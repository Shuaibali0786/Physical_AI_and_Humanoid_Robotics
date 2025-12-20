# Data Model: RAG Agent Frontend Integration

## Entity: User Query
- **Description**: The text input from the user containing their question about the book content
- **Fields**:
  - `text: string` - The actual question text
  - `timestamp: number` - When the query was submitted
  - `pageContext: PageContext` - Context of the current page (optional)
  - `metadata: object` - Additional metadata for the query

## Entity: Agent Response
- **Description**: The answer returned from the RAG agent, including text content and metadata
- **Fields**:
  - `answer: string` - The generated answer text
  - `query: string` - The original query that generated this response
  - `retrievedChunks: Array<RetrievedChunk>` - Chunks used to generate the response
  - `sources: Array<string>` - Sources referenced in the response
  - `confidence: number` - Confidence score (0-1)
  - `timestamp: number` - When the response was generated
  - `error: ErrorInfo` - Error information if the response failed (optional)

## Entity: Page Context
- **Description**: Information about the current book page including URL, title, and content snippets
- **Fields**:
  - `url: string` - Current page URL
  - `title: string` - Page title
  - `contentSnippet: string` - Relevant content snippet from the page (optional)
  - `section: string` - Section of the book (optional)

## Entity: Chat Session
- **Description**: The interaction state that maintains conversation history and loading states
- **Fields**:
  - `id: string` - Unique session identifier
  - `messages: Array<ChatMessage>` - Array of messages in the conversation
  - `isLoading: boolean` - Whether a query is currently being processed
  - `error: ErrorInfo` - Current error state (optional)
  - `pageContext: PageContext` - Context of the current page

## Entity: Chat Message
- **Description**: A single message in the chat conversation
- **Fields**:
  - `id: string` - Unique message identifier
  - `text: string` - The message content
  - `sender: 'user' | 'agent'` - Who sent the message
  - `timestamp: number` - When the message was created
  - `sources: Array<string>` - Sources referenced in agent responses (optional)

## Entity: Retrieved Chunk
- **Description**: Content chunk retrieved from Qdrant and used to generate the response
- **Fields**:
  - `id: string` - Chunk identifier
  - `content: string` - The actual content text
  - `score: number` - Relevance score
  - `url: string` - Source URL
  - `title: string` - Content title
  - `metadata: object` - Additional metadata

## Entity: Error Info
- **Description**: Information about errors that occur during the process
- **Fields**:
  - `type: string` - Type of error (e.g., 'network', 'api', 'validation')
  - `message: string` - Human-readable error message
  - `timestamp: number` - When the error occurred
  - `details: object` - Additional error details (optional)

## Entity: API Communication
- **Description**: The data exchange mechanism between frontend and backend including request/response formats
- **Fields**:
  - `endpoint: string` - API endpoint URL
  - `method: string` - HTTP method (GET, POST, etc.)
  - `headers: object` - Request headers
  - `requestBody: object` - Request payload
  - `response: object` - Response data
  - `status: number` - HTTP status code
  - `timestamp: number` - When the request was made
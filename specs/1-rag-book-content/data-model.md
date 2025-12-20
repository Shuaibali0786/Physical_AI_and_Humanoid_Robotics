# Data Model: RAG Book Content Processing System

## Book Content
**Description**: Represents the text content extracted from book URLs
**Fields**:
- url: string (URL of the book page)
- title: string (title of the page)
- content: string (extracted text content)
- content_type: string (type of content - text, code, etc.)
- created_at: datetime (timestamp when content was extracted)
- updated_at: datetime (timestamp when content was last updated)

## Text Chunk
**Description**: A segment of book content that has been processed and prepared for embedding generation
**Fields**:
- id: string (unique identifier for the chunk)
- content: string (text content of the chunk)
- source_url: string (URL of the original page)
- source_title: string (title of the original page)
- chunk_index: integer (position of chunk within original content)
- metadata: object (additional metadata about the chunk)

## Vector Embedding
**Description**: A numerical representation of text content that enables semantic similarity comparison
**Fields**:
- id: string (unique identifier for the embedding)
- vector: array<float> (numerical vector representation)
- content_id: string (reference to the source content)
- model: string (embedding model used)
- created_at: datetime (timestamp when embedding was generated)

## Vector Record
**Description**: A stored entry in the vector database containing the embedding, metadata, and original content reference
**Fields**:
- id: string (unique identifier for the record)
- payload: object (contains content, URL, title, and other metadata)
- vector: array<float> (the embedding vector)
- score: float (similarity score for search results)
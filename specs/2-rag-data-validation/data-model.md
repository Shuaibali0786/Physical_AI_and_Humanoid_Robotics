# Data Model: RAG Data Validation System

## Validation Report
**Description**: A structured output containing validation results, counts, and any identified issues
**Fields**:
- id: string (unique identifier for the validation run)
- timestamp: datetime (when validation was performed)
- collection_name: string (name of the Qdrant collection validated)
- total_chunks: integer (total number of chunks found in collection)
- total_embeddings: integer (total number of embeddings validated)
- valid_embeddings: integer (number of embeddings with correct dimensions)
- invalid_embeddings: integer (number of embeddings with incorrect dimensions)
- missing_content: integer (number of entries with missing content)
- corrupted_data: integer (number of entries with corrupted data)
- similarity_search_success_rate: float (percentage of successful similarity searches)
- issues: array<object> (detailed list of identified issues)
- summary: string (overall pass/fail status)

## Sample Query
**Description**: A predefined query used to test similarity search functionality with expected relevant results
**Fields**:
- id: string (unique identifier for the query)
- query_text: string (the text of the query)
- expected_url: string (URL expected to be in top results)
- expected_topic: string (topic that should be covered in results)
- category: string (category of the query for testing different aspects)

## Chunk Validation
**Description**: The process of verifying individual content chunks for completeness and correctness
**Fields**:
- chunk_id: string (identifier of the chunk being validated)
- url: string (URL of the source content)
- title: string (title of the content)
- content_length: integer (length of the content text)
- is_valid: boolean (whether the chunk passed validation)
- validation_errors: array<string> (list of validation errors found)

## Embedding Verification
**Description**: The process of validating that stored embeddings have correct dimensions and values
**Fields**:
- embedding_id: string (identifier of the embedding being validated)
- vector_dimension: integer (dimension of the embedding vector)
- is_valid_dimension: boolean (whether the dimension is correct)
- vector_values_valid: boolean (whether the values are valid floats)
- has_null_values: boolean (whether the vector contains null/empty values)
- validation_status: string (overall validation status)
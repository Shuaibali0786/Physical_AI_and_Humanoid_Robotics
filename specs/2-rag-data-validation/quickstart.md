# Quickstart: RAG Data Validation System

## Prerequisites
- Python 3.11+
- Qdrant Cloud account and API credentials
- Existing environment configuration from Spec-1 (ingestion pipeline)
- Access to the "physical_ai_book" collection in Qdrant

## Setup
1. Navigate to the backend directory
2. Install validation dependencies: `pip install -r requirements-validation.txt`
3. Ensure your .env file contains the same Qdrant configuration used in Spec-1

## Environment Variables
```
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_HOST=your_qdrant_host (if applicable)
QDRANT_PORT=6333
```

## Running the Validation
1. Execute the validation script: `python -m backend.validation.main`
2. The script will:
   - Connect to Qdrant using existing configuration
   - Retrieve all stored chunks from the "physical_ai_book" collection
   - Validate embedding dimensions and content integrity
   - Perform similarity search tests with sample queries
   - Generate a validation report

## Expected Output
- Validation report in JSON format showing:
  - Total chunks retrieved
  - Embedding validation results
  - Similarity search success rate
  - Any missing or corrupted data
  - Overall pass/fail status
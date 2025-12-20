# Function Contracts: RAG Book Content Processing System

## get_all_urls()
**Purpose**: Crawl the deployed site to discover all book page URLs
**Input**: base_url (string) - the base URL of the deployed book site
**Output**: List of URLs (array of strings)
**Error cases**: Network errors, invalid sitemap format, rate limiting

## extract_text_from_url()
**Purpose**: Extract clean text content from a given URL
**Input**: url (string) - the URL to extract content from
**Output**: Object with {url, title, content}
**Error cases**: Page not found, network errors, parsing errors

## create_embedding()
**Purpose**: Generate vector embedding for a text chunk using Cohere
**Input**: text (string) - the text to generate embedding for
**Output**: embedding vector (array of floats)
**Error cases**: Cohere API errors, rate limiting, invalid text

## create_collection()
**Purpose**: Create a collection in Qdrant Cloud for storing vectors
**Input**: collection_name (string) - name of the collection to create
**Output**: success status (boolean)
**Error cases**: Qdrant connection errors, collection already exists, insufficient permissions

## save_chunk_to_qdrant()
**Purpose**: Save a text chunk with its embedding to Qdrant Cloud
**Input**: chunk_data (object) - contains content, metadata, and embedding
**Output**: success status (boolean) and point ID (string)
**Error cases**: Qdrant connection errors, invalid data format, storage limits
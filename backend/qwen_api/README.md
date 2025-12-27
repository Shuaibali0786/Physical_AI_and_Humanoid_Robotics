# Groq RAG Chat API

This is a FastAPI-based backend service for the Groq-powered RAG chatbot that integrates with the Physical AI and Humanoid Robotics book content.

## Setup

1. Navigate to this directory:
   ```bash
   cd backend/qwen_api
   ```

2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   venv\Scripts\activate  # On Windows
   # source venv/bin/activate  # On macOS/Linux
   ```

3. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Create a `.env` file in this directory with your configuration:
   ```env
   GROQ_API_KEY=your_groq_api_key_here
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_COLLECTION_NAME=book_content
   ```

## Running the Server

### Option 1: Using the batch script (Windows)
```bash
start_server.bat
```

### Option 2: Manual start
```bash
cd backend/qwen_api
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

The server will start on `http://localhost:8000`

## API Endpoints

- `POST /api/qwen-chat` - Main chat endpoint
- `GET /health` - Health check endpoint

## Configuration

- The server runs on port 8000 by default
- The frontend expects this API to be available at `http://localhost:8000/api/qwen-chat`
- Make sure your Qdrant database contains the book content with Cohere embeddings
- Uses Groq API for language model responses (configured with GROQ_API_KEY)
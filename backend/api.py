"""
API Endpoints for Search and Translation Services
Provides REST API endpoints for both search and translation features
"""
import os
import logging
from typing import Dict, Any, List
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

from .search_service import search_book_content
from .translation_service import translate_text, TranslationResult

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI(title="Book Content API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class SearchRequest(BaseModel):
    query: str
    search_type: str = "vector"  # "vector", "keyword", "hybrid"
    top_k: int = 5

class SearchResponse(BaseModel):
    success: bool
    results: List[Dict[str, Any]]
    error: str = None

class TranslationRequest(BaseModel):
    text: str
    target_language: str = "roman_urdu"  # "roman_urdu" or "english"

class TranslationResponse(BaseModel):
    success: bool
    original_text: str
    translated_text: str
    target_language: str
    error: str = None

@app.get("/")
async def root():
    """Root endpoint to check if the API is running."""
    return {"message": "Book Content API is running", "version": "1.0.0"}

@app.post("/api/search", response_model=SearchResponse)
async def search_endpoint(request: SearchRequest):
    """Search endpoint to search book content."""
    try:
        logger.info(f"Received search request: {request.query}")

        results = search_book_content(
            query=request.query,
            search_type=request.search_type,
            top_k=request.top_k
        )

        logger.info(f"Search completed with {len(results)} results")

        return SearchResponse(
            success=True,
            results=results
        )
    except Exception as e:
        logger.error(f"Search error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/translate", response_model=TranslationResponse)
async def translate_endpoint(request: TranslationRequest):
    """Translation endpoint to translate text."""
    try:
        logger.info(f"Received translation request for language: {request.target_language}")

        result: TranslationResult = translate_text(
            text=request.text,
            target_language=request.target_language
        )

        if result.success:
            logger.info("Translation completed successfully")
            return TranslationResponse(
                success=True,
                original_text=result.original_text,
                translated_text=result.translated_text,
                target_language=request.target_language
            )
        else:
            logger.error(f"Translation failed: {result.error}")
            raise HTTPException(status_code=400, detail=result.error)
    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "service": "book-content-api"}

@app.get("/api/search/test")
async def test_search(query: str = Query(..., description="Search query")):
    """Test search endpoint with query parameter."""
    try:
        results = search_book_content(query=query)
        return SearchResponse(success=True, results=results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/translate/test")
async def test_translation(
    text: str = Query(..., description="Text to translate"),
    target_language: str = Query("roman_urdu", description="Target language")
):
    """Test translation endpoint with query parameters."""
    try:
        result: TranslationResult = translate_text(text=text, target_language=target_language)

        if result.success:
            return TranslationResponse(
                success=True,
                original_text=result.original_text,
                translated_text=result.translated_text,
                target_language=target_language
            )
        else:
            raise HTTPException(status_code=400, detail=result.error)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    port = int(os.getenv("PORT", 8001))  # Use a different port than the Qwen API
    uvicorn.run(app, host="0.0.0.0", port=port)
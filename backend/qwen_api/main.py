from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Optional, List, Dict
import os
import logging
from dotenv import load_dotenv
from fastapi.middleware.cors import CORSMiddleware

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Groq RAG Chat API",
    description="API for Groq-based RAG chatbot integrated with book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Request models
class ChatRequest(BaseModel):
    query: str
    context: Optional[str] = None
    target_language: str = "english"  # For translation support

class SearchRequest(BaseModel):
    query: str
    search_type: str = "hybrid"  # "vector", "keyword", "hybrid"
    top_k: int = 5

class TranslationRequest(BaseModel):
    text: str
    target_language: str = "roman_urdu"  # "roman_urdu" or "english"

# Response models
class ChatResponse(BaseModel):
    response: str
    sources: Optional[list] = []
    target_language: Optional[str] = "english"

class SearchResponse(BaseModel):
    success: bool
    results: List[Dict]
    error: Optional[str] = None

class TranslationResponse(BaseModel):
    success: bool
    original_text: str
    translated_text: str
    target_language: str
    error: Optional[str] = None

# Import the Groq RAG agent
try:
    from rag_qwen_agent import groq_rag_agent
except ImportError as e:
    # If the agent fails to import, create a mock version for development
    logger.warning(f"Could not import rag_qwen_agent: {e}. Using mock agent.")

    class MockGroqRAGAgent:
        def get_response(self, query: str, context: Optional[str] = None, target_language: str = "english") -> Dict:
            if "hi" in query.lower() or "hello" in query.lower():
                response_text = "Hello! I'm your Groq RAG Assistant. I can help you with questions about the book content. What would you like to know?"
            elif "ai" in query.lower():
                response_text = "AI (Artificial Intelligence) refers to computer systems designed to perform tasks that typically require human intelligence, such as learning, reasoning, problem-solving, and understanding natural language. In the context of Physical AI, it involves systems that interact with the physical world."
            elif "book" in query.lower() or "content" in query.lower():
                response_text = "This is a Physical AI and Humanoid Robotics book. I can help you understand concepts related to AI, robotics, and humanoid systems. What specific topic would you like to explore?"
            else:
                response_text = f"I received your query: '{query}'. I'm currently using a mock response system. In a full implementation, I would retrieve relevant information from the book content and generate a response using Groq."

            return {
                'response': response_text,
                'sources': [],
                'target_language': target_language
            }

        def search_content(self, query: str, search_type: str = "hybrid", top_k: int = 5) -> List[Dict]:
            # Mock search results
            if "ai" in query.lower():
                return [
                    {
                        'content': 'Artificial Intelligence (AI) refers to computer systems designed to perform tasks that typically require human intelligence, such as learning, reasoning, problem-solving, and understanding natural language. In Physical AI, these systems interact with the physical world.',
                        'url': '/module-1/ai-concepts',
                        'title': 'AI Concepts in Physical AI'
                    }
                ]
            elif "robot" in query.lower() or "humanoid" in query.lower():
                return [
                    {
                        'content': 'Humanoid robots are designed with a human-like body structure, including limbs and often a head, to interact effectively with human environments and tools.',
                        'url': '/module-2/humanoid-design',
                        'title': 'Humanoid Robot Design Principles'
                    }
                ]
            else:
                return [
                    {
                        'content': 'Physical AI and Humanoid Robotics is an interdisciplinary field combining artificial intelligence with robotics to create systems that can interact with the physical world in human-like ways.',
                        'url': '/module-1/introduction',
                        'title': 'Introduction to Physical AI and Humanoid Robotics'
                    }
                ]

        def translate_text(self, text: str, target_language: str = "roman_urdu") -> Dict:
            # Mock translation results
            return {
                'original_text': text,
                'translated_text': f"Mock translation of: {text}",
                'success': True,
                'error': None
            }

    groq_rag_agent = MockGroqRAGAgent()

@app.post("/api/qwen-chat", response_model=ChatResponse)
async def groq_chat(request: ChatRequest):
    """
    Main endpoint for the RAG chat functionality (using Groq API)
    """
    try:
        # Get response from Groq RAG agent
        result = groq_rag_agent.get_response(request.query, request.context, request.target_language)

        return ChatResponse(
            response=result['response'],
            sources=result.get('sources', []),
            target_language=result.get('target_language', 'english')
        )
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail="Error processing the query")

@app.post("/api/search", response_model=SearchResponse)
async def search_endpoint(request: SearchRequest):
    """
    Search endpoint for book content
    """
    try:
        # Perform search using the RAG agent
        results = qwen_rag_agent.search_content(request.query, request.search_type, request.top_k)

        return SearchResponse(
            success=True,
            results=results
        )
    except Exception as e:
        logger.error(f"Error performing search: {str(e)}")
        return SearchResponse(
            success=False,
            results=[],
            error=str(e)
        )

@app.post("/api/translate", response_model=TranslationResponse)
async def translate_endpoint(request: TranslationRequest):
    """
    Translation endpoint to convert text between English and Roman Urdu
    """
    try:
        # Perform translation using the RAG agent
        result = qwen_rag_agent.translate_text(request.text, request.target_language)

        return TranslationResponse(
            success=result['success'],
            original_text=result['original_text'],
            translated_text=result['translated_text'],
            target_language=request.target_language,
            error=result.get('error')
        )
    except Exception as e:
        logger.error(f"Error performing translation: {str(e)}")
        raise HTTPException(status_code=500, detail="Error performing translation")

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "Qwen RAG Chat API"}

@app.get("/api/test")
async def test_endpoint():
    """
    Simple test endpoint to verify the API is running
    """
    return {"message": "Qwen RAG API is running correctly", "status": "ok"}

@app.get("/api/search/test")
async def test_search(query: str):
    """
    Test search endpoint with query parameter
    """
    try:
        results = qwen_rag_agent.search_content(query)
        return SearchResponse(
            success=True,
            results=results
        )
    except Exception as e:
        logger.error(f"Error performing search: {str(e)}")
        return SearchResponse(
            success=False,
            results=[],
            error=str(e)
        )

@app.get("/api/translate/test")
async def test_translation(text: str, target_language: str = "roman_urdu"):
    """
    Test translation endpoint with query parameters
    """
    try:
        result = qwen_rag_agent.translate_text(text, target_language)
        return TranslationResponse(
            success=result['success'],
            original_text=result['original_text'],
            translated_text=result['translated_text'],
            target_language=target_language,
            error=result.get('error')
        )
    except Exception as e:
        logger.error(f"Error performing translation: {str(e)}")
        raise HTTPException(status_code=500, detail="Error performing translation")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
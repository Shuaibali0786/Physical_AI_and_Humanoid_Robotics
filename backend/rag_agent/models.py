"""
Pydantic models for request/response validation in RAG Agent
"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel


class QueryRequest(BaseModel):
    """
    Model for incoming query requests
    """
    query: str
    max_results: Optional[int] = 5
    temperature: Optional[float] = 0.7
    metadata_filters: Optional[Dict[str, Any]] = {}


class RetrievedChunk(BaseModel):
    """
    Model for content chunks retrieved from Qdrant
    """
    id: str
    content: str
    score: float
    url: str
    title: str
    metadata: Dict[str, Any] = {}


class AgentResponse(BaseModel):
    """
    Model for responses from the OpenAI Agent
    """
    answer: str
    query: str
    retrieved_chunks: List[RetrievedChunk]
    sources: List[str]
    confidence: float
    timestamp: float


class QueryResponse(BaseModel):
    """
    Model for API response
    """
    answer: str
    query: str
    retrieved_chunks: List[RetrievedChunk]
    sources: List[str]
    confidence: float
    timestamp: float


class HealthResponse(BaseModel):
    """
    Model for health check endpoint response
    """
    status: str
    details: Dict[str, Any]
    timestamp: float


class ConfigResponse(BaseModel):
    """
    Model for configuration endpoint response
    """
    qdrant_collection: str
    max_results_default: int
    temperature_default: float
    available_models: List[str]
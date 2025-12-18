"""
Pydantic models for API request/response validation.
"""

from pydantic import BaseModel
from typing import Optional, List


class Source(BaseModel):
    """Source document model."""
    title: str
    url: str
    relevance: float = 0.0


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str
    session_id: Optional[str] = None


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str
    sources: Optional[List[Source]] = None
    citations: Optional[List[str]] = None
    confidence: float = 0.5
    session_id: Optional[str] = None
    latency_ms: int = 0

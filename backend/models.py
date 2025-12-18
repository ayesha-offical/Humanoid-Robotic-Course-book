"""
Pydantic models for API request/response validation.
"""

from pydantic import BaseModel
from typing import Optional, List


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    message: str
    user_id: str = "guest"


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    response: str
    citations: Optional[List[str]] = None

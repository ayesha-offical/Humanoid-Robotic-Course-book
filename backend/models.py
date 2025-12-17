"""
Pydantic models for API request/response validation.
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str = Field(
        ...,
        min_length=5,
        max_length=2000,
        description="User query for the RAG agent (5-2000 characters)"
    )
    session_id: Optional[str] = Field(
        None,
        description="Optional session ID to group related queries"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do I install ROS 2?",
                "session_id": "session_123"
            }
        }


class Source(BaseModel):
    """Source reference in the response."""
    title: str = Field(..., description="Title of the source")
    url: str = Field(..., description="URL of the source")
    relevance: float = Field(..., ge=0.0, le=1.0, description="Relevance score (0-1)")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    answer: str = Field(..., description="Answer from the RAG agent")
    sources: List[Source] = Field(
        default_factory=list,
        description="List of sources used to generate the answer"
    )
    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score of the answer (0-1)"
    )
    session_id: Optional[str] = Field(
        None,
        description="Session ID for tracking conversation"
    )
    latency_ms: int = Field(..., ge=0, description="Response latency in milliseconds")

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 can be installed from the official repositories...",
                "sources": [
                    {
                        "title": "ROS 2 Installation Guide",
                        "url": "/docs/ros2-installation",
                        "relevance": 0.95
                    }
                ],
                "confidence": 0.92,
                "session_id": "session_123",
                "latency_ms": 1234
            }
        }


class HealthResponse(BaseModel):
    """Response model for health check endpoint."""
    status: str = Field(..., description="Health status: 'healthy', 'degraded', or 'unavailable'")
    uptime_seconds: int = Field(..., ge=0, description="Server uptime in seconds")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Current timestamp")
    version: str = Field(..., description="API version")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "uptime_seconds": 3600,
                "timestamp": "2025-12-18T12:00:00Z",
                "version": "0.1.0"
            }
        }


class ErrorResponse(BaseModel):
    """Response model for error responses."""
    error: str = Field(..., description="Error type/code")
    message: str = Field(..., description="Error message")
    status_code: int = Field(..., ge=400, le=599, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "validation_error",
                "message": "Query must be between 5 and 2000 characters",
                "status_code": 422,
                "timestamp": "2025-12-18T12:00:00Z"
            }
        }

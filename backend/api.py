"""
FastAPI application for RAG-powered chatbot API.
Exposes the RAG agent logic via REST API endpoints.
"""

import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from models import ChatRequest, ChatResponse
from rag_agent import agent
from agents import Runner


# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="REST API for RAG-powered chatbot",
    version="0.1.0",
    docs_url="/api/docs",
    openapi_url="/api/openapi.json"
)


# Configure CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://physcial-ai-and-humanoid-robotics-c.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Health check endpoint
@app.get("/")
async def root():
    """Root endpoint - health check status."""
    return {"status": "ok"}


# Chat endpoint
@app.post("/api/chat", response_model=ChatResponse)
def chat(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint that accepts a query and returns an answer.

    Uses Runner.run_sync() to invoke the RAG agent with the user's query.

    Args:
        request: ChatRequest with query and optional session_id

    Returns:
        ChatResponse with answer text, sources, and metadata
    """
    import time
    import logging

    logger = logging.getLogger(__name__)
    start_time = time.time()

    try:
        # Call the agent using Runner.run_sync() from agents SDK
        result = Runner.run_sync(agent, input=request.query)

        # Extract the final output from the result
        answer_text = result.final_output if hasattr(result, 'final_output') else str(result)

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        logger.info(f"Chat query: {request.query[:50]}... -> latency: {latency_ms}ms")

        # Return ChatResponse with the agent's response
        return ChatResponse(
            answer=answer_text,
            sources=None,
            citations=None,
            confidence=0.8,
            session_id=request.session_id,
            latency_ms=latency_ms
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        return ChatResponse(
            answer=f"Error: {str(e)}. Please try again.",
            sources=None,
            citations=None,
            confidence=0.0,
            session_id=request.session_id,
            latency_ms=int((time.time() - start_time) * 1000)
        )


if __name__ == "__main__":
    import uvicorn

    # Run the app with uvicorn
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000
    )

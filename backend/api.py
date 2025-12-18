"""
FastAPI application for RAG-powered chatbot API.
Exposes the RAG agent logic via REST API endpoints.
"""

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
    Chat endpoint that accepts a message and returns a response.

    Uses Runner.run_sync() to invoke the RAG agent with the user's message.

    Args:
        request: ChatRequest with message and optional user_id

    Returns:
        ChatResponse with response text and optional citations
    """
    # Call the agent using Runner.run_sync() from agents SDK
    result = Runner.run_sync(agent, input=request.message)

    # Extract the final output from the result
    response_text = result.final_output if hasattr(result, 'final_output') else str(result)

    # Return ChatResponse with the agent's response
    return ChatResponse(response=response_text, citations=None)


if __name__ == "__main__":
    import uvicorn

    # Run the app with uvicorn
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000
    )

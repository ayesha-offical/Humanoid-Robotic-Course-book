import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from dotenv import load_dotenv
import logging

from agent import run_agent, save_chat_history

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Robotics Course API",
    description="FastAPI backend for RAG-based course chatbot",
    version="1.0.0"
)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for hackathon
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request and Response Models
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    user_id: str


class ChatResponse(BaseModel):
    response: str
    source: str  # "rag" or "selected_text"


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "ok", "message": "Server is running"}


@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that accepts a message and optional selected text.

    - If selected_text is provided, use it as context (ignore RAG)
    - If selected_text is empty, use RAG (Qdrant search)
    """
    try:
        # Validate inputs
        if not request.message or not request.message.strip():
            raise HTTPException(
                status_code=400,
                detail="Message cannot be empty"
            )

        if not request.user_id or not request.user_id.strip():
            raise HTTPException(
                status_code=400,
                detail="User ID is required"
            )

        # Determine context source and run agent
        if request.selected_text and request.selected_text.strip():
            logger.info(f"Using selected text for user {request.user_id}")
            response_text = run_agent(
                question=request.message,
                selected_text=request.selected_text,
                use_rag=False
            )
            source = "selected_text"
        else:
            logger.info(f"Using RAG for user {request.user_id}")
            response_text = run_agent(
                question=request.message,
                selected_text=None,
                use_rag=True
            )
            source = "rag"

        # Save chat history to Neon DB (non-blocking, handles failures gracefully)
        try:
            save_chat_history(
                user_id=request.user_id,
                query=request.message,
                response=response_text,
                source=source
            )
            logger.info(f"Chat history saved for user {request.user_id}")
        except Exception as db_error:
            logger.warning(f"Failed to save chat history: {db_error}")
            # Don't fail the request if DB is down

        return ChatResponse(response=response_text, source=source)

    except HTTPException as http_exc:
        logger.error(f"HTTP Error: {http_exc.detail}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in /chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    logger.info(f"Starting FastAPI server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)

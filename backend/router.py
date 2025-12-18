"""
FastAPI router for chat endpoints.
Integrates the RAG agent logic.
"""

import time
import logging
from fastapi import APIRouter, HTTPException, Request
from models import ChatRequest, ChatResponse, Source
from config import settings
from rag_agent import agent
from agents import Runner
from embeding_helpers import embed
from qdrant_client import QdrantClient

logger = logging.getLogger(__name__)

# Initialize router
router = APIRouter(prefix="/api", tags=["chat"])

# Initialize Qdrant client for source retrieval
qdrant_client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
    check_compatibility=False
)


async def query_rag_agent(query: str) -> dict:
    """
    Query the RAG agent with a given query.

    Args:
        query: User query string

    Returns:
        Dictionary with answer, sources, and confidence

    Raises:
        TimeoutError: If query takes longer than configured timeout
        Exception: If agent fails
    """
    try:
        logger.info(f"Querying RAG agent with: {query[:100]}...")

        # Call the agent using Runner.run_sync() from agents SDK
        # This is the proper way to invoke the agent with the OpenAI Agent SDK
        result = Runner.run_sync(agent, input=query)

        # Extract answer from final_output
        # Runner.run_sync() returns a result object with final_output field
        answer = result.final_output if hasattr(result, 'final_output') else str(result)

        logger.info(f"RAG agent response: {answer[:100]}...")

        # Retrieve sources from Qdrant based on the query
        sources = await retrieve_sources(query)

        # Calculate confidence based on response length and sources
        confidence = min(0.95, max(0.5, len(sources) / 5.0))

        return {
            "answer": answer,
            "sources": sources,
            "confidence": confidence
        }

    except Exception as e:
        logger.error(f"Error querying RAG agent: {str(e)}")
        raise


async def retrieve_sources(query: str, limit: int = 5) -> list:
    """
    Retrieve source documents from Qdrant based on query similarity.

    Args:
        query: User query string
        limit: Maximum number of sources to retrieve

    Returns:
        List of Source objects
    """
    try:
        # Generate embedding for the query
        query_embedding = embed(query)

        # Search Qdrant for similar documents
        results = qdrant_client.query_points(
            collection_name=settings.collection_name,
            query=query_embedding,
            limit=limit
        )

        # Convert results to Source objects
        sources = []
        for idx, point in enumerate(results.points):
            payload = point.payload
            source = Source(
                title=payload.get("title", f"Source {idx + 1}"),
                url=payload.get("url", f"/docs/source-{point.id}"),
                relevance=min(1.0, (limit - idx) / limit)  # Relevance decreases with rank
            )
            sources.append(source)

        logger.info(f"Retrieved {len(sources)} sources for query")
        return sources

    except Exception as e:
        logger.warning(f"Error retrieving sources: {str(e)}")
        return []


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint - Query the RAG agent and return structured response.

    Args:
        request: ChatRequest with query and optional session_id

    Returns:
        ChatResponse with answer, sources, confidence, and latency

    Raises:
        HTTPException: 422 for validation errors
        HTTPException: 500 for server errors
    """
    start_time = time.time()

    try:
        # Validate query length
        if len(request.query) < settings.min_query_length:
            logger.warning(f"Query too short: {len(request.query)} chars")
            raise HTTPException(
                status_code=422,
                detail=f"Query must be at least {settings.min_query_length} characters"
            )

        if len(request.query) > settings.max_query_length:
            logger.warning(f"Query too long: {len(request.query)} chars")
            raise HTTPException(
                status_code=422,
                detail=f"Query must not exceed {settings.max_query_length} characters"
            )

        logger.info(f"Received chat request: {request.query[:50]}... (session: {request.session_id})")

        # Query the RAG agent
        agent_response = await query_rag_agent(request.query)

        # Calculate latency
        latency_ms = int((time.time() - start_time) * 1000)

        # Check for timeout
        if latency_ms > (settings.request_timeout_seconds * 1000):
            logger.error(f"Request exceeded timeout: {latency_ms}ms")
            raise HTTPException(
                status_code=504,
                detail="Request timeout - query took too long to process"
            )

        # Build response
        response = ChatResponse(
            answer=agent_response["answer"],
            sources=agent_response["sources"],
            confidence=agent_response["confidence"],
            session_id=request.session_id,
            latency_ms=latency_ms
        )

        logger.info(f"Chat request completed: {latency_ms}ms, confidence: {response.confidence:.2f}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Internal server error - failed to process query"
        )


@router.get("/chat/health")
async def chat_health() -> dict:
    """
    Health check endpoint for chat service.

    Returns:
        Dictionary with service status
    """
    try:
        # Try to connect to Qdrant
        qdrant_client.get_collections()
        qdrant_status = "healthy"
    except Exception as e:
        logger.warning(f"Qdrant connection failed: {str(e)}")
        qdrant_status = "unavailable"

    return {
        "status": "healthy" if qdrant_status == "healthy" else "degraded",
        "qdrant": qdrant_status,
        "message": "Chat service is operational"
    }

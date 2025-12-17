"""
Main FastAPI application entry point.
Configures CORS, middleware, error handling, and mounts routers.
"""

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
import time
import logging
from datetime import datetime

from config import settings, validate_settings, logger
from router import router
from models import HealthResponse, ErrorResponse

# Validate settings before creating app
if not validate_settings():
    logger.warning("Settings validation failed - some features may not work")

# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    description="RAG-powered chatbot API for Physical AI & Humanoid Robotics Textbook",
    version=settings.app_version,
    docs_url="/api/docs",
    openapi_url="/api/openapi.json",
)

# Track app startup time
app.start_time = time.time()


# CORS Middleware - Allow Docosaurus frontend to access API
app.add_middleware(
    CORSMiddleware,
    **settings.get_cors_config()
)


# Custom middleware for request/response logging
@app.middleware("http")
async def log_requests(request: Request, call_next):
    """Log incoming requests and outgoing responses."""
    start_time = time.time()
    request_id = request.headers.get("x-request-id", "unknown")

    # Log request
    logger.info(f"[{request_id}] {request.method} {request.url.path}")

    try:
        response = await call_next(request)
    except Exception as e:
        logger.error(f"[{request_id}] Exception: {str(e)}", exc_info=True)
        raise

    # Log response
    duration = time.time() - start_time
    logger.info(f"[{request_id}] {response.status_code} ({duration:.3f}s)")

    return response


# Global exception handler for validation errors
@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    """Handle validation errors with custom response format."""
    logger.warning(f"Validation error: {exc.errors()}")

    error_response = ErrorResponse(
        error="validation_error",
        message=f"Request validation failed: {exc.error_count()} error(s)",
        status_code=422,
    )

    return JSONResponse(
        status_code=422,
        content=error_response.dict()
    )


# Mount routers
app.include_router(router)


# Health check endpoints
@app.get("/", response_model=dict)
async def root():
    """Root endpoint - API information."""
    return {
        "name": settings.app_name,
        "version": settings.app_version,
        "status": "operational",
        "docs": "/api/docs",
        "docs_alternative": "/api/redoc"
    }


@app.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint - returns API health status."""
    uptime = int(time.time() - app.start_time)

    return HealthResponse(
        status="healthy",
        uptime_seconds=uptime,
        version=settings.app_version
    )


@app.get("/api/health", response_model=HealthResponse)
async def api_health_check() -> HealthResponse:
    """Alternative health check endpoint at /api/health."""
    uptime = int(time.time() - app.start_time)

    return HealthResponse(
        status="healthy",
        uptime_seconds=uptime,
        version=settings.app_version
    )


# Startup events
@app.on_event("startup")
async def startup_event():
    """Run on application startup."""
    logger.info(f"Starting {settings.app_name} v{settings.app_version}")
    logger.info(f"Debug mode: {settings.debug}")
    logger.info(f"CORS origins: {settings.cors_origins}")


@app.on_event("shutdown")
async def shutdown_event():
    """Run on application shutdown."""
    logger.info(f"Shutting down {settings.app_name}")


# Error handler for general exceptions
@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle unexpected exceptions."""
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)

    error_response = ErrorResponse(
        error="internal_server_error",
        message="An unexpected error occurred",
        status_code=500,
    )

    return JSONResponse(
        status_code=500,
        content=error_response.dict()
    )


if __name__ == "__main__":
    import uvicorn

    # Run with uvicorn if executed directly
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level=settings.log_level.lower()
    )

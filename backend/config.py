"""
Configuration management for FastAPI backend.
Uses Pydantic Settings for type-safe environment variable loading.
"""

import os
from pydantic_settings import BaseSettings
from typing import List
import logging


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # API Configuration
    app_name: str = "Physical AI Chatbot API"
    app_version: str = "0.1.0"
    debug: bool = False

    # OpenAI Configuration
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")

    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    collection_name: str = os.getenv("COLLECTION_NAME", "curriculum_embeddings")

    # Request Configuration
    request_timeout_seconds: int = 30
    max_query_length: int = 2000
    min_query_length: int = 5

    # CORS Configuration
    cors_origins: List[str] = [
        "http://localhost:3000",      # Docosaurus dev
        "http://localhost:8000",      # FastAPI dev
        "http://127.0.0.1:3000",      # Docosaurus local
        "http://127.0.0.1:8000",      # FastAPI local
    ]
    cors_methods: List[str] = ["GET", "POST", "OPTIONS"]
    cors_headers: List[str] = ["Content-Type", "Authorization"]

    # Rate Limiting
    rate_limit_requests_per_minute: int = 60

    # Logging Configuration
    log_level: str = "INFO"
    log_format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    class Config:
        env_file = ".env"
        case_sensitive = False

    def get_cors_config(self) -> dict:
        """Get CORS configuration dictionary for FastAPI."""
        return {
            "allow_origins": self.cors_origins,
            "allow_credentials": True,
            "allow_methods": self.cors_methods,
            "allow_headers": self.cors_headers,
        }


# Global settings instance
settings = Settings()


# Configure logging
def configure_logging():
    """Configure application logging."""
    logging.basicConfig(
        level=getattr(logging, settings.log_level),
        format=settings.log_format
    )
    return logging.getLogger(__name__)


logger = configure_logging()


# Validation
def validate_settings() -> bool:
    """Validate critical settings are present."""
    required_settings = [
        ("OPENAI_API_KEY", settings.openai_api_key),
        ("QDRANT_URL", settings.qdrant_url),
        ("QDRANT_API_KEY", settings.qdrant_api_key),
    ]

    missing = []
    for name, value in required_settings:
        if not value:
            missing.append(name)

    if missing:
        logger.warning(f"Missing required settings: {', '.join(missing)}")
        return False

    logger.info(f"Settings validated successfully. App: {settings.app_name} v{settings.app_version}")
    return True

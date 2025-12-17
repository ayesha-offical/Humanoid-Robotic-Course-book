"""
Unit tests for FastAPI endpoints.
Tests request validation, response structure, error handling.
"""

import pytest
from fastapi.testclient import TestClient
from api import app
from models import ChatRequest, ChatResponse, HealthResponse


@pytest.fixture
def client():
    """Provide FastAPI test client."""
    return TestClient(app)


class TestHealthEndpoints:
    """Test health check endpoints."""

    def test_root_endpoint(self, client):
        """Test GET / returns API information."""
        response = client.get("/")
        assert response.status_code == 200
        data = response.json()
        assert "name" in data
        assert "version" in data
        assert "status" in data
        assert data["status"] == "operational"

    def test_health_check_root(self, client):
        """Test GET /health returns health status."""
        response = client.get("/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "uptime_seconds" in data
        assert data["uptime_seconds"] >= 0

    def test_health_check_api(self, client):
        """Test GET /api/health returns health status."""
        response = client.get("/api/health")
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "uptime_seconds" in data


class TestChatEndpoint:
    """Test chat endpoint."""

    def test_chat_valid_query(self, client):
        """Test POST /api/chat with valid query returns ChatResponse."""
        payload = {
            "query": "How do I install ROS 2?",
            "session_id": "test_session_1"
        }
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()

        # Verify response structure
        assert "answer" in data
        assert "sources" in data
        assert "confidence" in data
        assert "session_id" in data
        assert "latency_ms" in data

        # Verify types
        assert isinstance(data["answer"], str)
        assert isinstance(data["sources"], list)
        assert isinstance(data["confidence"], (int, float))
        assert isinstance(data["latency_ms"], int)

        # Verify values
        assert len(data["answer"]) > 0
        assert 0 <= data["confidence"] <= 1
        assert data["latency_ms"] >= 0
        assert data["session_id"] == "test_session_1"

    def test_chat_without_session_id(self, client):
        """Test POST /api/chat works without session_id."""
        payload = {"query": "What is ROS 2?"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()
        assert data["session_id"] is None

    def test_chat_query_too_short(self, client):
        """Test POST /api/chat rejects query shorter than 5 chars."""
        payload = {"query": "hi"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 422
        data = response.json()
        assert "error" in data
        assert data["error"] == "validation_error"

    def test_chat_query_too_long(self, client):
        """Test POST /api/chat rejects query longer than 2000 chars."""
        long_query = "a" * 2001
        payload = {"query": long_query}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 422

    def test_chat_missing_query(self, client):
        """Test POST /api/chat fails when query is missing."""
        payload = {}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 422
        data = response.json()
        assert "error" in data

    def test_chat_response_structure(self, client):
        """Test POST /api/chat response has all required fields."""
        payload = {"query": "Explain ROS 2 concepts"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()

        # Validate ChatResponse schema
        chat_response = ChatResponse(**data)
        assert chat_response.answer is not None
        assert isinstance(chat_response.sources, list)
        assert 0 <= chat_response.confidence <= 1
        assert chat_response.latency_ms >= 0

    def test_chat_sources_structure(self, client):
        """Test POST /api/chat response sources have correct structure."""
        payload = {"query": "What is sim-to-real transfer?"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()

        # Check sources if present
        if data["sources"]:
            for source in data["sources"]:
                assert "title" in source
                assert "url" in source
                assert "relevance" in source
                assert isinstance(source["relevance"], (int, float))
                assert 0 <= source["relevance"] <= 1

    def test_chat_latency_reasonable(self, client):
        """Test POST /api/chat completes in reasonable time."""
        payload = {"query": "Quick question"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()
        # Should complete within 30 seconds
        assert data["latency_ms"] < 30000


class TestCORSHeaders:
    """Test CORS configuration."""

    def test_cors_headers_present(self, client):
        """Test that CORS headers are included in response."""
        response = client.options("/api/chat")

        # Check for CORS headers
        assert "access-control-allow-origin" in response.headers or \
               "Access-Control-Allow-Origin" in response.headers

    def test_cors_methods(self, client):
        """Test that CORS allows correct methods."""
        response = client.options("/api/chat")

        if "access-control-allow-methods" in response.headers:
            allowed_methods = response.headers["access-control-allow-methods"].upper()
            assert "POST" in allowed_methods


class TestErrorHandling:
    """Test error handling."""

    def test_invalid_json(self, client):
        """Test POST /api/chat with invalid JSON returns error."""
        response = client.post(
            "/api/chat",
            content="invalid json {",
            headers={"Content-Type": "application/json"}
        )
        assert response.status_code in [400, 422]

    def test_wrong_http_method(self, client):
        """Test GET /api/chat returns method not allowed."""
        response = client.get("/api/chat")
        assert response.status_code == 405

    def test_nonexistent_endpoint(self, client):
        """Test GET /api/nonexistent returns 404."""
        response = client.get("/api/nonexistent")
        assert response.status_code == 404


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

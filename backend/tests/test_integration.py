"""
Integration tests for FastAPI application.
Tests end-to-end flows and component interaction.
"""

import pytest
import time
from fastapi.testclient import TestClient
from api import app


@pytest.fixture
def client():
    """Provide FastAPI test client."""
    return TestClient(app)


class TestChatIntegration:
    """Integration tests for chat functionality."""

    def test_chat_complete_flow(self, client):
        """Test complete chat flow: request -> processing -> response."""
        # Test 1: Send query
        payload = {
            "query": "What is ROS 2?",
            "session_id": "integration_test_1"
        }
        response = client.post("/api/chat", json=payload)

        # Verify successful response
        assert response.status_code == 200
        data = response.json()

        # Verify response has all expected fields
        assert "answer" in data
        assert "sources" in data
        assert "confidence" in data
        assert "session_id" in data
        assert "latency_ms" in data

        # Verify session tracking
        assert data["session_id"] == "integration_test_1"

    def test_multiple_queries_in_session(self, client):
        """Test multiple queries in same session."""
        session_id = "integration_test_session"

        queries = [
            "How do I install ROS 2?",
            "What is a ROS node?",
            "Explain ROS topics",
        ]

        responses = []
        for query in queries:
            payload = {
                "query": query,
                "session_id": session_id
            }
            response = client.post("/api/chat", json=payload)
            assert response.status_code == 200
            responses.append(response.json())

        # Verify all responses have the same session_id
        for response_data in responses:
            assert response_data["session_id"] == session_id

    def test_query_variety(self, client):
        """Test various types of queries."""
        test_queries = [
            "What is ROS 2?",                      # Simple question
            "How do I install ROS 2 on Ubuntu?",   # Installation query
            "Explain sim-to-real transfer",        # Concept explanation
            "What hardware do I need?",             # Hardware query
        ]

        for query in test_queries:
            payload = {"query": query}
            response = client.post("/api/chat", json=payload)

            # Each query should return 200
            assert response.status_code == 200, f"Failed for query: {query}"

            # Response should be valid
            data = response.json()
            assert "answer" in data
            assert len(data["answer"]) > 0

    def test_response_contains_sources(self, client):
        """Test that responses include source information when available."""
        payload = {"query": "Teach me about ROS 2 fundamentals"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()

        # Should have sources
        assert "sources" in data
        assert isinstance(data["sources"], list)

    def test_confidence_scoring(self, client):
        """Test that confidence scores are meaningful."""
        payload = {"query": "What is ROS 2?"}
        response = client.post("/api/chat", json=payload)

        assert response.status_code == 200
        data = response.json()

        # Confidence should be between 0 and 1
        assert 0 <= data["confidence"] <= 1

    def test_latency_tracking(self, client):
        """Test that response latency is tracked accurately."""
        payload = {"query": "Explain ROS 2 architecture"}

        start_time = time.time()
        response = client.post("/api/chat", json=payload)
        actual_duration = (time.time() - start_time) * 1000

        assert response.status_code == 200
        data = response.json()

        # Latency should be positive and reasonable
        assert data["latency_ms"] > 0
        # API latency should be close to actual duration (within 10% margin)
        assert data["latency_ms"] <= actual_duration * 1.1

    def test_error_recovery(self, client):
        """Test that API recovers from errors gracefully."""
        # Send invalid query (too short)
        response1 = client.post(
            "/api/chat",
            json={"query": "hi"}
        )
        assert response1.status_code == 422

        # Next valid query should work fine
        response2 = client.post(
            "/api/chat",
            json={"query": "How does ROS 2 work?"}
        )
        assert response2.status_code == 200


class TestHealthIntegration:
    """Integration tests for health monitoring."""

    def test_health_endpoints_consistency(self, client):
        """Test that both health endpoints return consistent status."""
        response1 = client.get("/health")
        response2 = client.get("/api/health")

        assert response1.status_code == 200
        assert response2.status_code == 200

        data1 = response1.json()
        data2 = response2.json()

        # Both should report healthy
        assert data1["status"] == "healthy"
        assert data2["status"] == "healthy"

    def test_uptime_increases(self, client):
        """Test that reported uptime increases over time."""
        response1 = client.get("/health")
        uptime1 = response1.json()["uptime_seconds"]

        time.sleep(0.1)

        response2 = client.get("/health")
        uptime2 = response2.json()["uptime_seconds"]

        # Uptime should increase or stay same
        assert uptime2 >= uptime1


class TestCORSIntegration:
    """Integration tests for CORS configuration."""

    def test_cors_preflight_request(self, client):
        """Test that CORS preflight requests are handled."""
        response = client.options(
            "/api/chat",
            headers={
                "Origin": "http://localhost:3000",
                "Access-Control-Request-Method": "POST",
            }
        )

        # Should return success for preflight
        assert response.status_code == 200

    def test_cors_allowed_origin(self, client):
        """Test that allowed origins can access the API."""
        # Test with localhost:3000 (Docosaurus dev server)
        response = client.post(
            "/api/chat",
            json={"query": "What is ROS 2?"},
            headers={"Origin": "http://localhost:3000"}
        )

        assert response.status_code == 200

    def test_response_content_type(self, client):
        """Test that responses have correct content type."""
        response = client.post(
            "/api/chat",
            json={"query": "What is ROS 2?"}
        )

        assert response.status_code == 200
        assert "application/json" in response.headers.get("content-type", "")


class TestPerformance:
    """Performance-related integration tests."""

    def test_concurrent_simulated_requests(self, client):
        """Test API behavior with multiple sequential requests."""
        latencies = []

        for i in range(5):
            start = time.time()
            response = client.post(
                "/api/chat",
                json={"query": f"Query number {i}: What is ROS 2?"}
            )
            latency = (time.time() - start) * 1000

            assert response.status_code == 200
            latencies.append(latency)

        # Average latency should be reasonable
        avg_latency = sum(latencies) / len(latencies)
        assert avg_latency < 30000  # 30 seconds

    def test_large_query_handling(self, client):
        """Test API can handle large queries (up to max length)."""
        large_query = "Explain " + " ".join([f"concept_{i}" for i in range(100)])

        response = client.post(
            "/api/chat",
            json={"query": large_query}
        )

        # Should handle large query successfully
        if len(large_query) <= 2000:
            assert response.status_code == 200
        else:
            assert response.status_code == 422


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

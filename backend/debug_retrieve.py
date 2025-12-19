"""
Debug script to see what the retrieve function returns for different queries
"""

import os
from dotenv import load_dotenv
from embeding_helpers import embed
from qdrant_client import QdrantClient

load_dotenv()

qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
collection_name = os.getenv("COLLECTION_NAME")

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    check_compatibility=False
)

test_queries = [
    "tell me about module 1",
    "module 1",
    "ROS2",
    "What is ROS2 architecture",
    "Tell me about ROS2 architecture packages nodes topics"
]

print("=" * 70)
print("DEBUG: Retrieving content for different queries")
print("=" * 70)

for query in test_queries:
    print(f"\nQuery: '{query}'")
    print("-" * 70)

    try:
        embedding = embed(query)
        result = qdrant.query_points(
            collection_name=collection_name,
            query=embedding,
            limit=3
        )

        if result.points:
            print(f"Found {len(result.points)} results:\n")
            for idx, point in enumerate(result.points):
                text = point.payload.get("text", "")[:150]
                url = point.payload.get("url", "")
                print(f"[{idx+1}] {text}...")
                print(f"     URL: {url}\n")
        else:
            print("NO RESULTS FOUND!")

    except Exception as e:
        print(f"Error: {e}")

print("=" * 70)
print("If 'tell me about module 1' returns empty or poor results,")
print("that's why the agent says 'I don't know'.")
print("=" * 70)

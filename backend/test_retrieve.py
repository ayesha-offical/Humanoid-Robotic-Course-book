"""
Test if the retrieve function can find data in Qdrant.
"""

import os
from dotenv import load_dotenv
from embeding_helpers import embed
from qdrant_client import QdrantClient

load_dotenv()

qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
collection_name = os.getenv("COLLECTION_NAME")

# Connect to Qdrant
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    check_compatibility=False
)

print("=" * 60)
print("Qdrant Retrieve Test")
print("=" * 60)

# Test queries
test_queries = [
    "How do I install ROS 2?",
    "What is ROS 2?",
    "ROS installation",
    "Physical AI",
    "Humanoid robots"
]

print(f"\nCollection: {collection_name}")
print(f"Testing {len(test_queries)} queries...\n")

for query in test_queries:
    print(f"Query: '{query}'")
    try:
        # Generate embedding
        embedding = embed(query)
        print(f"  Embedding generated: {len(embedding)} dimensions")

        # Query Qdrant
        results = qdrant.query_points(
            collection_name=collection_name,
            query=embedding,
            limit=5
        )

        print(f"  Results found: {len(results.points)}")

        if len(results.points) > 0:
            for i, point in enumerate(results.points):
                score = point.score if hasattr(point, 'score') else "N/A"
                text = point.payload.get("text", "")[:100] + "..."
                print(f"    [{i+1}] Score: {score}")
                print(f"         Text: {text}")
        else:
            print(f"    ✗ No results found!")

    except Exception as e:
        print(f"  ✗ Error: {e}")

    print()

print("=" * 60)

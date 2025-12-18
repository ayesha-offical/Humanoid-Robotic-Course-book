"""
Test Qdrant connection and configuration.
"""

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

print("=" * 60)
print("Qdrant Connection Test")
print("=" * 60)

# Check environment variables
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("COLLECTION_NAME")

print(f"\n[CONFIG]")
print(f"  URL: {qdrant_url}")
print(f"  API Key: {qdrant_api_key[:20]}..." if qdrant_api_key else "  API Key: NOT SET")
print(f"  Collection: {collection_name}")

# Try to connect
print(f"\n[CONNECTING]")
try:
    qdrant = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        check_compatibility=False
    )
    print("✓ Connected to Qdrant successfully")

    # Try to get collections
    print(f"\n[CHECKING COLLECTIONS]")
    try:
        collections = qdrant.get_collections()
        print(f"✓ Found {len(collections.collections)} collection(s):")
        for col in collections.collections:
            print(f"  - {col.name} ({col.points_count} points)")
    except Exception as e:
        print(f"✗ Error listing collections: {e}")

except Exception as e:
    print(f"✗ Failed to connect to Qdrant: {e}")
    print("\nPossible issues:")
    print("  1. Qdrant Cloud instance is not running")
    print("  2. API key is incorrect")
    print("  3. URL is incorrect or unreachable")
    print("\nSolution:")
    print("  - Check your .env file: QDRANT_URL and QDRANT_API_KEY")
    print("  - Verify Qdrant Cloud instance is active at https://cloud.qdrant.io")
    print("  - Or run local Qdrant: docker run -p 6333:6333 qdrant/qdrant")

print("\n" + "=" * 60)

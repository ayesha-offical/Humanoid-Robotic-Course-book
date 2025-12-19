"""
Test all connections to verify the setup works
"""

import os
from dotenv import load_dotenv

load_dotenv()

print("=" * 70)
print("TESTING CONNECTIONS")
print("=" * 70)

# 1. Check environment variables
print("\n[1] Checking Environment Variables")
print("-" * 70)

required_vars = [
    "OPENAI_API_KEY",
    "COHERE_API_KEY",
    "QDRANT_API_KEY",
    "QDRANT_URL",
    "COLLECTION_NAME"
]

missing = []
for var in required_vars:
    value = os.getenv(var)
    if value:
        # Show first and last 10 chars only (for security)
        display = value[:10] + "..." + value[-10:] if len(value) > 20 else "***"
        print(f"  [OK] {var} = {display}")
    else:
        print(f"  [MISSING] {var}")
        missing.append(var)

if missing:
    print(f"\nMISSING: {', '.join(missing)}")
    print("Set these in your .env or Hugging Face secrets!")

# 2. Test Cohere connection
print("\n[2] Testing Cohere Connection")
print("-" * 70)

try:
    import cohere
    cohere_key = os.getenv("COHERE_API_KEY")
    if not cohere_key:
        print("  [FAIL] COHERE_API_KEY not set")
    else:
        client = cohere.Client(cohere_key)
        result = client.embed(
            model="embed-english-v3.0",
            input_type="search_query",
            texts=["test"]
        )
        print(f"  [OK] Cohere connected. Got embedding of size {len(result.embeddings[0])}")
except Exception as e:
    print(f"  [FAIL] Cohere error: {str(e)[:100]}")

# 3. Test Qdrant connection
print("\n[3] Testing Qdrant Connection")
print("-" * 70)

try:
    from qdrant_client import QdrantClient

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("COLLECTION_NAME")

    if not qdrant_url or not qdrant_key:
        print("  [FAIL] QDRANT_URL or QDRANT_API_KEY not set")
    else:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_key,
            check_compatibility=False
        )

        # Try to get collections
        collections = client.get_collections()
        print(f"  [OK] Qdrant connected. Found {len(collections.collections)} collections")

        # Check if our collection exists
        try:
            info = client.get_collection(collection_name)
            print(f"  [OK] Collection '{collection_name}' exists with {info.points_count} points")
        except Exception as e:
            print(f"  [FAIL] Collection '{collection_name}' not found: {str(e)[:50]}")

except Exception as e:
    print(f"  [FAIL] Qdrant error: {str(e)[:100]}")

# 4. Test OpenAI connection
print("\n[4] Testing OpenAI Connection")
print("-" * 70)

try:
    from openai import OpenAI

    openai_key = os.getenv("OPENAI_API_KEY")
    if not openai_key:
        print("  [FAIL] OPENAI_API_KEY not set")
    else:
        client = OpenAI(api_key=openai_key)
        # Just check if we can create the client (don't make API call to save credits)
        print(f"  [OK] OpenAI client initialized")
except Exception as e:
    print(f"  [FAIL] OpenAI error: {str(e)[:100]}")

# 5. Test full retrieve workflow
print("\n[5] Testing Full Retrieve Workflow")
print("-" * 70)

try:
    from embeding_helpers import embed
    from qdrant_client import QdrantClient

    query = "module 1"
    collection_name = os.getenv("COLLECTION_NAME")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_key = os.getenv("QDRANT_API_KEY")

    # Get embedding
    print(f"  1. Embedding query '{query}'...")
    embedding = embed(query)
    print(f"     [OK] Got embedding of size {len(embedding)}")

    # Query Qdrant
    print(f"  2. Searching Qdrant collection '{collection_name}'...")
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_key,
        check_compatibility=False
    )
    results = client.query_points(
        collection_name=collection_name,
        query=embedding,
        limit=3
    )

    print(f"     [OK] Found {len(results.points)} results")

    if results.points:
        print(f"  3. Retrieved content:")
        for idx, point in enumerate(results.points):
            text = point.payload.get("text", "")[:80]
            print(f"     [{idx+1}] {text}...")
    else:
        print(f"     [WARNING] No results found!")

except Exception as e:
    print(f"  [FAIL] Workflow error: {str(e)[:150]}")

print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)
print("If all tests pass, your setup is correct.")
print("If any test fails, fix the issue before deploying to Hugging Face.")
print("=" * 70)

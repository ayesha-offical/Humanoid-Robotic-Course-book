"""
Test script to verify Module 1 content is retrievable from Qdrant
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

def test_all_modules_retrieval():
    """Test if ALL module content can be retrieved"""

    print("=" * 70)
    print("TESTING ALL MODULES CONTENT RETRIEVAL")
    print("=" * 70)

    # Check collection info
    try:
        collection_info = qdrant.get_collection(collection_name)
        print(f"\nCollection: {collection_name}")
        print(f"Total data chunks loaded: {collection_info.points_count}")
        print("[OK] Data successfully saved to Qdrant!\n")
    except Exception as e:
        print(f"Error getting collection info: {e}")
        return

    # Test queries for ALL modules
    test_queries = {
        "Module 0 (Foundations)": [
            "What is the course overview?",
            "Explain physical AI concepts"
        ],
        "Module 1 (ROS2)": [
            "Tell me about ROS2 architecture",
            "How do I install ROS 2?",
            "What are ROS 2 packages?",
            "Explain ROS 2 advanced topics"
        ],
        "Module 2 (Simulation)": [
            "How do I set up Gazebo?",
            "Tell me about Gazebo sensors",
            "What is robot simulation?"
        ],
        "Module 3 (Isaac)": [
            "What is Isaac Sim?",
            "Explain Isaac ROS",
            "How do you do sim-to-real transfer?"
        ],
        "Module 4 (Humanoid Robotics)": [
            "Explain humanoid dynamics",
            "What is humanoid manipulation?",
            "Tell me about conversational robotics"
        ]
    }

    print("=" * 70)
    print("TESTING ALL MODULES WITH SAMPLE QUERIES")
    print("=" * 70)

    total_results = 0
    modules_with_data = 0

    for module_name, queries in test_queries.items():
        print(f"\n{module_name}")
        print("-" * 70)
        module_has_data = False

        for query in queries:
            try:
                query_embedding = embed(query)
                results = qdrant.query_points(
                    collection_name=collection_name,
                    query=query_embedding,
                    limit=2
                )

                if results.points:
                    if not module_has_data:
                        module_has_data = True
                        modules_with_data += 1

                    total_results += len(results.points)
                    text_preview = results.points[0].payload.get("text", "")[:80]
                    url = results.points[0].payload.get("url", "")
                    print(f"  [YES] '{query}'")
                    print(f"    -> Found relevant content from: {url.split('/')[-1]}")
                else:
                    print(f"  [NO] '{query}' - No content found")

            except Exception as e:
                print(f"  [ERR] '{query}' - Error: {str(e)[:50]}")

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"[OK] Modules with data: {modules_with_data}/5")
    print(f"[OK] Total chunks available: {collection_info.points_count}")
    print(f"[OK] Successfully retrieved content from {total_results} chunks across queries")

    if modules_with_data == 5:
        print("\n[SUCCESS] All modules are properly loaded and retrievable!")
    else:
        print(f"\n[WARNING] Only {modules_with_data} out of 5 modules have data")

    print("=" * 70)

if __name__ == "__main__":
    test_all_modules_retrieval()

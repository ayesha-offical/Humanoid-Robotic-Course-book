import cohere
from qdrant_client import QdrantClient
from embeding_helpers import embed
import os
from dotenv import load_dotenv
load_dotenv()

qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")


# Connect to Qdrant Cloud
qdrant = QdrantClient(
url=qdrant_url, 
api_key=qdrant_api_key,
)


def data_retrieve(query):
    embedding = embed(query)
    result = qdrant.query_points(
        collection_name="physical_humanoid_ai_book",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]

# Test
print(data_retrieve("What data do you have?"))
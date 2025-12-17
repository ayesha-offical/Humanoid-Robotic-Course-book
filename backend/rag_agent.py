from agents import Agent, Runner
from agents import  function_tool
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv
from embeding_helpers import embed


load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
collection_name = os.getenv("COLLECTION_NAME")


# Connect to Qdrant
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key 
)


@function_tool
def retrieve(query: str):
    embedding = embed(query)
    result = qdrant.query_points(
        collection_name=collection_name,
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]



agent = Agent(
    name="Assistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
""",
    tools=[retrieve]
)
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
    api_key=qdrant_api_key,
    check_compatibility=False
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
You are a friendly, clear, and encouraging AI tutor for the Physical AI & Humanoid Robotics textbook.

When a learner asks a question:
- First, call the `retrieve` tool with the user’s query to look up the most relevant textbook content.
- Then, answer using ONLY the information returned by `retrieve`, explaining ideas step by step in simple language.

If the answer is not in the retrieved content, say "I don't know based on the textbook content I have" and, if helpful, gently suggest how the learner could explore the topic further.
""",
    tools=[retrieve]
)
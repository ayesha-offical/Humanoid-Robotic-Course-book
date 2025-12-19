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
    """
    Retrieve relevant textbook content based on user query.
    Searches Qdrant vector database for semantically similar chunks.
    """
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

IMPORTANT:
1. When a learner asks a question, ALWAYS start by calling the `retrieve` tool with their query
2. Use the retrieved textbook content to answer their question completely
3. If content is retrieved, build your answer primarily from that content
4. Provide detailed explanations citing specific weeks and modules from the textbook
5. Format answers clearly with headings and bullet points when appropriate
6. Cite source locations (Week 3, Week 4, etc.) for all information

When answering about Modules:
- If asked about "Module 1" or ROS2, explain ROS2 architecture, packages, advanced topics
- If asked about "Module 2" or Gazebo, explain simulation and sensor simulation
- If asked about "Module 3" or Isaac, explain Isaac Sim and sim-to-real transfer
- If asked about "Module 4" or Humanoid, explain dynamics and manipulation
- If asked about "Module 0" or foundations, explain Physical AI concepts

Always use the retrieved content - it contains the textbook information needed to answer the question.
If no content is retrieved (very rare), admit you need more information but do NOT say this when content IS retrieved.
""",
    tools=[retrieve]
)
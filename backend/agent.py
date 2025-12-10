import os
from openai import OpenAI
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
import asyncpg
import logging
from datetime import datetime

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 1. Setup Environment
load_dotenv()
# Make sure .env mein OPENAI_API_KEY mojood hai
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# 2. Setup Cohere & Qdrant (Aapki Keys)
cohere_client = cohere.Client("CRPmtkTBubmFr7O9bpJyIPX2uNS347M8wy11UQJd")
qdrant = QdrantClient(
    url="https://8fda3468-1811-4e03-ab6a-28c4ec0c2a0f.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0._xXiPjwexglperAdMm9bGCfT-nyt9OwkP9hu9P8TWRE",
)

# NOTE: Collection name wahi hona chahiye jo ingest.py mein use kiya tha
COLLECTION_NAME = "humanoid_ai_course_book"

# 3. Neon Database Configuration
DATABASE_URL = os.getenv("DATABASE_URL", None) 

# 4. Helper Functions
def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]


def retrieve(query):
    """Retrieve context chunks from Qdrant using RAG"""
    logger.info(f"🔍 Searching knowledge base for: '{query}'...")
    embedding = get_embedding(query)

    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )
    # Return text from retrieved points
    return [point.payload["text"] for point in result.points]


# 5. Database Functions
async def init_chat_history_table():
    """Create chat_history table if it doesn't exist"""
    if not DATABASE_URL:
        logger.warning("DATABASE_URL not set. Skipping table initialization.")
        return

    try:
        conn = await asyncpg.connect(DATABASE_URL)
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                user_id VARCHAR(255) NOT NULL,
                query TEXT NOT NULL,
                response TEXT NOT NULL,
                source VARCHAR(50) NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );

            CREATE INDEX IF NOT EXISTS idx_user_id ON chat_history(user_id);
            CREATE INDEX IF NOT EXISTS idx_created_at ON chat_history(created_at);
        """)
        await conn.close()
        logger.info("Chat history table initialized successfully")
    except Exception as e:
        logger.warning(f"Failed to initialize chat history table: {e}")


def save_chat_history(user_id: str, query: str, response: str, source: str):
    """
    Save chat interaction to Neon Postgres (synchronous wrapper).
    Handles failures gracefully without blocking the chat.
    """
    if not DATABASE_URL:
        logger.debug("DATABASE_URL not set. Skipping chat history save.")
        return

    try:
        import asyncio
        asyncio.run(_save_chat_history_async(user_id, query, response, source))
    except Exception as e:
        logger.warning(f"Failed to save chat history: {e}")


async def _save_chat_history_async(user_id: str, query: str, response: str, source: str):
    """Async function to save chat history"""
    try:
        conn = await asyncpg.connect(DATABASE_URL)
        await conn.execute("""
            INSERT INTO chat_history (user_id, query, response, source, created_at)
            VALUES ($1, $2, $3, $4, $5)
        """, user_id, query, response, source, datetime.utcnow())
        await conn.close()
        logger.info(f"Chat history saved for user {user_id}")
    except Exception as e:
        logger.error(f"Error saving to database: {e}")
        raise


# 6. Main Agent Logic
def run_agent(question: str, selected_text: str = None, use_rag: bool = True):
    """
    Run the AI agent with flexible context handling.

    Args:
        question: The user's question
        selected_text: Optional text selected by the user
        use_rag: Whether to use RAG (Qdrant) for context

    Returns:
        The AI response
    """
    # Determine context based on selected_text vs RAG
    if selected_text and not use_rag:
        # Use selected text as ONLY context
        logger.info("Using selected text as context")
        context_text = selected_text
        source_info = "(from selected text)"
    elif use_rag:
        # Use RAG to retrieve context
        logger.info("Using RAG to retrieve context")
        context_chunks = retrieve(question)
        context_text = "\n\n".join(context_chunks)
        source_info = "(from knowledge base)"
    else:
        # No context available
        context_text = ""
        source_info = ""

    # System Instructions
    system_prompt = """
    You are the specialized AI Tutor for the 'Physical AI & Humanoid Robotics' textbook.
    Your goal is to help students master complex topics like ROS 2, NVIDIA Isaac Sim, Gazebo, and VLA (Vision-Language-Action) systems.

    INSTRUCTIONS:
    1. **Strict Context Adherence:** Answer the user's question using ONLY the provided 'Context' text. Do not use outside knowledge to answer questions that are not in the book.
    2. **Educational Tone:** Explain concepts clearly as a professor would. If the context contains code, explain what the code is doing.
    3. **No Hallucinations:** If the provided context does not contain the answer, you must reply: "I'm sorry, but this specific information is not currently available in the textbook modules."
    4. **Structure:** Use Bullet points or Bold text to make the answer easy to read.
    5. **Role:** Act as a helpful mentor. Encourage the student if the topic is difficult.
    """

    # Call OpenAI
    logger.info("🤖 Generating answer...")
    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {question}"}
        ],
        temperature=0
    )

    return response.choices[0].message.content


# 7. Main entry point
if __name__ == "__main__":
    # Test RAG mode
    output = run_agent("tell me about 13 week course detail", use_rag=True)
    print(output)
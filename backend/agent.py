import os
from openai import OpenAI
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
import asyncpg
import logging
from datetime import datetime
import json

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 1. Setup Environment
load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# 2. Setup Cohere & Qdrant
cohere_client = cohere.Client("CRPmtkTBubmFr7O9bpJyIPX2uNS347M8wy11UQJd")
qdrant = QdrantClient(
    url="https://8fda3468-1811-4e03-ab6a-28c4ec0c2a0f.us-east4-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0._xXiPjwexglperAdMm9bGCfT-nyt9OwkP9hu9P8TWRE",
)

COLLECTION_NAME = "humanoid_ai_course_book"
DATABASE_URL = os.getenv("DATABASE_URL", None) 

# 3. Helper Functions
def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]


def search_textbook(query: str) -> str:
    """
    Tool: Search the textbook knowledge base.

    This function is called by the AI agent to retrieve relevant context
    from the Qdrant vector database when answering questions.

    Args:
        query: The search query

    Returns:
        Combined text from the 5 most relevant chunks
    """
    logger.info(f"🔍 Agent calling search_textbook tool for: '{query}'")
    embedding = get_embedding(query)

    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )

    chunks = [point.payload["text"] for point in result.points]
    combined_context = "\n\n".join(chunks)
    logger.info(f"Retrieved {len(chunks)} chunks from knowledge base")

    return combined_context


def retrieve(query):
    """Retrieve context chunks from Qdrant (legacy function)"""
    logger.info(f"🔍 Searching knowledge base for: '{query}'...")
    embedding = get_embedding(query)

    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )
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


# 4. Tool Definition for OpenAI Function Calling
SEARCH_TOOL = {
    "type": "function",
    "function": {
        "name": "search_textbook",
        "description": "Search the Physical AI & Humanoid Robotics textbook knowledge base to find relevant content for answering student questions",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query to find relevant textbook sections (e.g., 'ROS 2 installation', 'Isaac Sim tutorial')"
                }
            },
            "required": ["query"]
        }
    }
}


# 5. TextbookAgent Class - Agentic RAG Implementation
class TextbookAgent:
    """
    An agentic AI tutor that autonomously decides whether to search the textbook
    based on the user's question and selected context.

    Uses OpenAI's Function Calling (Tools) to enable the agent to decide when
    to search the knowledge base versus using user-provided context.
    """

    def __init__(self):
        self.client = client
        self.model = "gpt-4o-mini"
        self.system_prompt = """
You are the specialized AI Tutor for the 'Physical AI & Humanoid Robotics' textbook.
Your goal is to help students master complex topics like ROS 2, NVIDIA Isaac Sim, Gazebo, and VLA (Vision-Language-Action) systems.

INSTRUCTIONS:
1. **Autonomy**: You can autonomously decide to call the search_textbook tool to find relevant context.
2. **Context Adherence**: Answer ONLY using information from the textbook (retrieved or provided). Do not use outside knowledge.
3. **Educational Tone**: Explain concepts clearly as a professor would. If context contains code, explain what it does.
4. **Handling Missing Info**: If you cannot find the answer in the textbook, reply: "I'm sorry, but this specific information is not currently available in the textbook modules."
5. **Structure**: Use bullet points or bold text to make answers easy to read.
6. **Mentorship**: Act as a helpful mentor. Encourage students when topics are difficult.
"""
        self.selected_text_prompt = """
You are the specialized AI Tutor for the 'Physical AI & Humanoid Robotics' textbook.

IMPORTANT: The user has provided their own context below. You MUST use ONLY this provided text to answer.
DO NOT call the search_textbook tool. Instead, base your answer entirely on what the user gave you.

If the provided text doesn't contain the answer, say: "This topic is not covered in the provided text."

INSTRUCTIONS:
1. **Use Only Provided Text**: Answer based ONLY on the text provided by the user.
2. **Educational Tone**: Explain concepts clearly.
3. **Structure**: Use bullet points or bold text.
4. **No Tool Calls**: Do NOT call search_textbook.
"""

    def think(self, question: str, selected_text: str = None) -> tuple[str, str]:
        """
        The agent's main thinking loop with tool execution.

        Args:
            question: The user's question
            selected_text: Optional user-provided context

        Returns:
            Tuple of (response_text, source) where source is "rag" or "selected_text"
        """
        logger.info("🤖 Agent starting think loop...")

        # Build the system prompt based on context mode
        if selected_text:
            logger.info("Agent operating in SELECTED_TEXT mode (tools disabled)")
            system_prompt = self.selected_text_prompt
            user_message = f"{self.selected_text_prompt}\n\nPROVIDED TEXT:\n{selected_text}\n\nSTUDENT QUESTION: {question}"
            tools = None  # No tools allowed when using selected text
            source = "selected_text"
        else:
            logger.info("Agent operating in RAG mode (can call search_textbook tool)")
            system_prompt = self.system_prompt
            user_message = question
            tools = [SEARCH_TOOL]  # Allow tool use
            source = "rag"

        # Initialize message history
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]

        # Agentic loop - keep running until we get a final response
        max_iterations = 5
        iteration = 0

        while iteration < max_iterations:
            iteration += 1
            logger.info(f"Iteration {iteration}/{max_iterations}")

            # Call OpenAI with or without tools
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=tools,
                temperature=0
            )

            # Check if model wants to use a tool
            if response.stop_reason == "tool_calls":
                logger.info("Agent decided to call a tool")

                # Process each tool call
                for tool_call in response.message.tool_calls:
                    tool_name = tool_call.function.name
                    tool_args = json.loads(tool_call.function.arguments)

                    logger.info(f"Tool called: {tool_name} with args: {tool_args}")

                    if tool_name == "search_textbook":
                        # Execute the search tool
                        search_query = tool_args.get("query")
                        tool_result = search_textbook(search_query)
                        logger.info(f"Tool returned {len(tool_result)} characters of context")

                        # Add assistant response and tool result to messages
                        messages.append({"role": "assistant", "content": response.message.content})
                        messages.append({
                            "role": "user",
                            "content": f"Tool result from search_textbook:\n\n{tool_result}"
                        })
                    else:
                        logger.warning(f"Unknown tool: {tool_name}")
                        break

            else:
                # Model returned a final response (stop_reason == "end_turn")
                logger.info("Agent generated final response")
                final_response = response.message.content
                return final_response, source

        # Max iterations reached
        logger.warning("Max iterations reached in agent loop")
        return "I encountered an error processing your question. Please try again.", source


# 6. Backward-compatible run_agent function
def run_agent(question: str, selected_text: str = None, use_rag: bool = True) -> str:
    """
    Backward-compatible wrapper that uses the TextbookAgent.

    Args:
        question: The user's question
        selected_text: Optional text selected by the user
        use_rag: Whether to use RAG (kept for compatibility, ignored if selected_text provided)

    Returns:
        The AI response
    """
    agent = TextbookAgent()

    if selected_text:
        response, source = agent.think(question, selected_text=selected_text)
    else:
        response, source = agent.think(question, selected_text=None)

    # Log the source for debugging
    logger.info(f"Response source: {source}")

    return response


# 7. Main entry point
if __name__ == "__main__":
    # Test Agentic RAG mode
    agent = TextbookAgent()
    output, source = agent.think("Tell me about the 13-week course structure")
    print(f"\n{'='*60}")
    print(f"Source: {source}")
    print(f"{'='*60}")
    print(output)
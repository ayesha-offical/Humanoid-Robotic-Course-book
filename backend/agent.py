import os
from openai import OpenAI
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient


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

# NOTE: Collection name wahi hona chahiye jo main.py mein use kiya tha
COLLECTION_NAME = "humanoid_ai_course_book" 

# 3. Helper Functions (Logic same as before)
def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]


def retrieve(query):
    """Qdrant se data uthane wala tool"""
    print(f"🔍 Searching knowledge base for: '{query}'...")
    embedding = get_embedding(query)
    
    result = qdrant.query_points(
        collection_name=COLLECTION_NAME,
        query=embedding,
        limit=5
    )
    # Text wapas karo
    return [point.payload["text"] for point in result.points]

# 4. Main Agent Logic (Replaces the 'Agent' class)
def run_agent(question):
    # Step A: Retrieve Context (Tool Call)
    context_chunks = retrieve(question)
    context_text = "\n\n".join(context_chunks)
    
    # Step B: System Instructions
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

    # Step C: Call OpenAI
    print("🤖 Generating answer...")
    response = client.chat.completions.create(
        model="gpt-4o-mini", # OpenAI Model
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {question}"}
        ],
        temperature=0
    )
    
    return response.choices[0].message.content

# 5. Run it
if __name__ == "__main__":
    final_output = run_agent("tell me about 13 week course detail")
    print(final_output)
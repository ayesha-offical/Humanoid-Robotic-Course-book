from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import os
import trafilatura
import xml.etree.ElementTree as ET
import cohere
import requests
from dotenv import load_dotenv
load_dotenv()
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
cohere_api_key = os.getenv("COHERE_API_KEY")

COLLECTION_NAME = os.getenv("COLLECTION_NAME")


cohere_client = cohere.Client(cohere_api_key)
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
url=qdrant_url, 
    api_key=qdrant_api_key,
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    try:
        xml = requests.get(sitemap_url).text
        root = ET.fromstring(xml)

        urls = []
        for child in root:
            loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if loc_tag is not None:
                urls.append(loc_tag.text)
        
        return urls
    except Exception as e:
        print(f"Error fetching sitemap: {e}")
        return []

# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    try:
        html = requests.get(url, timeout=10).text # Timeout added
        text = trafilatura.extract(html)
        if not text:
            print(f"[SKIP] No text extracted from: {url}")
        return text
    except Exception as e:
        print(f"[ERROR] Failed to extract {url}: {e}")
        return None

# -------------------------------------
# Step 3 — Chunk the text (SAFE VERSION)
# -------------------------------------
def chunk_text(text, max_chars=1000):
    """
    Fixed function to prevent MemoryError (Infinite Loop).
    """
    if not text:
        return []
    
    chunks = []
    start = 0
    text_len = len(text)
    
    while start < text_len:
        end = start + max_chars
        
        
        if end >= text_len:
            chunks.append(text[start:])
            break
     
        split_pos = text.rfind('. ', start, end)
        
        if split_pos == -1:
            split_pos = text.rfind(' ', start, end)
            
    
        if split_pos == -1:
            split_pos = end
        else:
            split_pos += 1 # Include the dot/space

        chunk = text[start:split_pos].strip()
        
        
        if chunk:
            chunks.append(chunk)
        
        # Move start pointer forward
            start = end
        else:
            start = split_pos
            
    return chunks

# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]
    
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    # Using specific config to avoid errors
    try:
        qdrant.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1024,
                distance=Distance.COSINE
            )
        )
        print("Collection created successfully.")
    except Exception as e:
        print(f"Collection creation warning (might exist): {e}")

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)
    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )
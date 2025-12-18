# Book Data Ingestion Guide

## Problem Identified
Your chat system was returning "I couldn't retrieve the details" because the book modules weren't loaded into the vector database (Qdrant). The RAG system needs actual content to retrieve and provide answers.

## Solution Implemented

### What We Fixed
1. **Added Rate Limiting** - Modified `main.py` to respect Cohere API's 40 calls/minute limit
2. **Fixed Unicode Encoding** - Fixed error message display issues
3. **Automated Ingestion** - The system now:
   - Fetches all URLs from your Docosaurus sitemap
   - Extracts text from each page
   - Splits into chunks (1000 chars max)
   - Embeds chunks using Cohere API
   - Stores embeddings in Qdrant vector database

## Data Flow

```
Docosaurus Site (humanoid-robotic-course-book.vercel.app)
         ↓ (Scrape via sitemap)
Text Extraction (using Trafilatura)
         ↓ (Extract textbook content)
Chunking (1000 chars per chunk)
         ↓ (Break into searchable pieces)
Embedding (Cohere API - 1024 dimensions)
         ↓ (Convert to vectors)
Qdrant Cloud Vector Database
         ↓ (Semantic search)
User Queries → RAG System → AI Response
```

## Current Status

### Ingestion Running
The `main.py` script is currently loading all book content:

**Book Structure:**
- Module 0: Foundations (Course Overview, Physical AI concepts)
- Module 1: ROS2 (Architecture, Packages, Advanced topics) ← THIS IS WHAT YOU ASKED ABOUT
- Module 2: Simulation (Gazebo Setup, Sensors)
- Module 3: Isaac (Isaac Sim, Isaac ROS, Sim-to-Real)
- Module 4: Humanoid Robotics (Dynamics, Manipulation, Conversational Robotics)

**Progress:**
- ✓ Module 0 content loaded
- ✓ Module 1 content being loaded
- ⏳ Modules 2-4 in progress
- Rate limited to 1.5 seconds per chunk to avoid API throttling

## How to Verify Data Was Loaded

### Method 1: Run Test Script (After Ingestion Completes)
```bash
cd backend
python test_module1.py
```

This will:
1. Check total points in Qdrant
2. Test 5 different Module 1 queries
3. Show retrieved chunks and their sources

### Method 2: Check Qdrant Collection Count
```python
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    check_compatibility=False
)
info = qdrant.get_collection(os.getenv("COLLECTION_NAME"))
print(f"Total chunks in database: {info.points_count}")
```

## Testing the Fix

Once ingestion completes:

### Test 1: Ask About Module 1
```
Query: "Tell me about Module 1"
Expected: Details about ROS2 architecture, packages, advanced concepts
```

### Test 2: Ask Specific Questions
```
Query: "What is ROS2 architecture?"
Expected: Information about nodes, topics, services, actions, messages

Query: "How do I install ROS 2?"
Expected: Installation steps and requirements

Query: "What are ROS 2 packages?"
Expected: Explanation of packages and their structure
```

### Test 3: Check Confidence Scores
The API now returns:
- **answer**: The AI response
- **sources**: Top 5 retrieved chunks with URLs
- **confidence**: 0.0-1.0 (higher = more relevant data found)

If confidence < 0.5, it means limited source material was found.

## API Response Structure

```json
{
  "answer": "Module 1 covers ROS 2...",
  "sources": [
    {
      "title": "Source 1",
      "url": "https://humanoid-robotic-course-book.vercel.app/docs/module-1-ros2/...",
      "relevance": 1.0
    },
    ...
  ],
  "confidence": 0.95,
  "session_id": "optional-session-123",
  "latency_ms": 2150
}
```

## Troubleshooting

### Issue: Still Getting "Can't retrieve details"
**Solution:**
1. Check total points: `python test_module1.py`
2. If < 50 points, re-run: `python main.py`
3. Check API keys in `.env` are valid
4. Verify Qdrant connectivity: Check dashboard at https://cloud.qdrant.io

### Issue: Slow Ingestion
**Reason:** 1.5 second delay per chunk respects Cohere API limits
**Timeline:** ~5-10 minutes for full ingestion
**Option:** If you upgrade to paid Cohere API, increase rate or use batch embedding

### Issue: Specific Module Not Appearing
**Solution:** Run ingestion again with `python main.py`
The script will:
- Recreate the collection (deletes old data)
- Scrape and reload all modules fresh
- Skip pages that failed and continue

## Key Files Modified

1. **main.py** - Added rate limiting and better error handling
2. **test_module1.py** - NEW - Verification script
3. **.env** - REQUIRES: COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL

## Next Steps

1. **Wait for ingestion to complete** (~5-10 minutes)
2. **Run test script**: `python test_module1.py`
3. **Test chat API** with Module 1 queries
4. **Monitor confidence scores** - if too low, re-ingest
5. **Optional**: Consider upgrading Cohere API key for faster ingestion

## Production Recommendations

- **Periodic Re-ingestion**: Run `main.py` monthly to keep content fresh
- **Monitor API Costs**: Track Cohere embedding calls
- **Cache Embeddings**: Store embeddings to avoid re-embedding same content
- **Upgrade Cohere Key**: Move from Trial (40 req/min) to Production for better performance
- **Add Content Version Control**: Track which content version is in Qdrant

## Questions?

Check the following:
- `.env` file has all required keys
- Cohere API key is valid (check at https://dashboard.cohere.com/)
- Qdrant is accessible (check at https://cloud.qdrant.io/)
- Network connection is stable (ingestion requires web scraping)

---

**Status**: Ingestion in progress. Run `python test_module1.py` once complete to verify all modules are retrievable.

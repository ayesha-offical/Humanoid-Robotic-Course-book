# Quick Start Guide - FastAPI RAG Backend

Get your hackathon backend running in 3 steps!

## Step 1: Install Dependencies

```bash
pip install -r requirements.txt
```

**Installs:**
- FastAPI web framework
- Uvicorn ASGI server
- OpenAI, Cohere, Qdrant clients
- asyncpg for Neon Postgres
- Other utilities

## Step 2: Configure Environment

Edit `.env` file:

```env
OPENAI_API_KEY=your_openai_key_here
DATABASE_URL=postgresql://user:pass@host/db    # Optional
HOST=0.0.0.0
PORT=8000
```

**Required:** `OPENAI_API_KEY`
**Optional:** `DATABASE_URL` (chat works without it)

## Step 3: Run the Server

```bash
python server.py
```

**Output:**
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

---

## Test the API

### Option 1: Swagger UI (Easiest)
Open your browser: `http://localhost:8000/docs`

Click "Try it out" on the `/chat` endpoint:
```json
{
  "message": "What is ROS 2?",
  "user_id": "test_user"
}
```

### Option 2: cURL
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Tell me about the course",
    "user_id": "student1"
  }'
```

### Option 3: Python
```python
import requests

response = requests.post(
    "http://localhost:8000/chat",
    json={
        "message": "What is NVIDIA Isaac Sim?",
        "user_id": "test"
    }
)
print(response.json())
```

---

## API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/health` | GET | Check server status |
| `/chat` | POST | Send message & get response |
| `/docs` | GET | Interactive API documentation |

---

## Two Chat Modes

### Mode 1: RAG (Default)
```json
{
  "message": "What is VLA in robotics?",
  "user_id": "student1"
}
```
→ Server searches Qdrant for relevant course content

### Mode 2: Selected Text
```json
{
  "message": "Explain this concept",
  "selected_text": "VLA stands for Vision-Language-Action...",
  "user_id": "student1"
}
```
→ Server uses only the selected_text as context

---

## Database Setup (Optional)

To save chat history:

1. Create Neon Postgres at https://neon.tech (free)
2. Copy connection string
3. Update `.env`:
   ```env
   DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/neondb
   ```
4. First `/chat` request auto-creates the `chat_history` table

**Without DATABASE_URL?** Chat still works fine - just no history saved!

---

## Common Issues

### ❌ "ModuleNotFoundError: No module named 'fastapi'"
**Solution:** Run `pip install -r requirements.txt`

### ❌ "OPENAI_API_KEY not found"
**Solution:** Add your key to `.env` file

### ❌ "Connection refused" from Qdrant
**Solution:** Make sure Qdrant cloud credentials are in `agent.py` (they already are!)

### ❌ Database connection fails
**Solution:** This is OK! Chat still works. DATABASE_URL is optional.

---

## Deployment Options

### Local Testing
```bash
python server.py
```

### Production with Gunicorn
```bash
pip install gunicorn
gunicorn -w 4 -k uvicorn.workers.UvicornWorker server:app
```

### Heroku Deployment
```bash
# Create Procfile
echo "web: gunicorn -w 4 -k uvicorn.workers.UvicornWorker server:app" > Procfile

# Deploy
git push heroku main
```

### Railway/Render/Replit
1. Connect your GitHub repo
2. Set environment variables
3. Deploy! (they auto-detect Python)

---

## File Structure

```
backend/
├── server.py              ← FastAPI app (START HERE)
├── agent.py              ← RAG logic
├── ingest.py             ← Data ingestion
├── requirements.txt      ← Dependencies
├── .env                  ← Config
├── README.md             ← Full documentation
└── QUICKSTART.md         ← You are here!
```

---

## Next Steps

1. ✅ Install & run the server
2. ✅ Test endpoints via Swagger UI
3. ✅ Connect your frontend
4. ✅ (Optional) Set up Neon database
5. ✅ Deploy!

---

## Support

- **Full Docs:** See `README.md`
- **Implementation Details:** See `IMPLEMENTATION_SUMMARY.md`
- **Code:** Check `server.py` and `agent.py`

Good luck with your hackathon! 🚀

---

**Pro Tip:** The Swagger UI at `http://localhost:8000/docs` is your best friend for testing!

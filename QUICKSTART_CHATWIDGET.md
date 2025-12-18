# ChatWidget Quick Start

## TL;DR - Get It Running in 3 Steps

### Step 1: Backend
```bash
cd backend/
python -m uvicorn api:app --reload
# Now listening on http://localhost:8000
```

### Step 2: Frontend
```bash
cd docosaurus/
npm install
npm run start
# Now open http://localhost:3000
```

### Step 3: Test
- Click the 💬 button (bottom-right corner)
- Type "Hello" and press Enter
- See the response (or error if backend not configured)

---

## What You'll See

```
┌─────────────────────────────────┐
│ AI Course Assistant         ✕    │  ← Header with close button
├─────────────────────────────────┤
│ Bot: Hello! I'm your AI...      │  ← Messages
│                                 │
│ You: What is ROS 2?             │
│ Bot: ROS 2 is...                │
│      Sources: [source1] [src2]  │
├─────────────────────────────────┤
│ [Input field] [Send]            │  ← Input area
└─────────────────────────────────┘
  ↑
  Floating button (💬) in bottom-right
```

---

## Environment Setup

**Create `docosaurus/.env.local`**:
```env
REACT_APP_API_URL=http://localhost:8000/api/chat
```

**Create `backend/.env`**:
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=...
```

---

## Backend API Contract

**Endpoint**: `POST /api/chat`

**Request**:
```json
{
  "query": "string (required)",
  "session_id": "string (optional)"
}
```

**Response** (200):
```json
{
  "answer": "AI response text",
  "sources": ["file1.md", "file2.md"],
  "citations": ["https://link1", "https://link2"],
  "confidence": 0.95,
  "latency_ms": 1500
}
```

**Error** (any error):
```
Error will display in chat as:
"Error: [error description]. Check backend at http://localhost:8000/api/chat"
```

---

## Features at a Glance

| Feature | How to Use |
|---------|-----------|
| Open Chat | Click 💬 button |
| Send Message | Press Enter or click Send |
| New Line | Press Shift+Enter |
| Close Chat | Click ✕ or 💬 button again |
| View Sources | Scroll down in message (sources appear below response) |
| Dark Mode | Enable in Docosaurus (automatic) |
| Mobile | Tap 💬 button, works full-width |

---

## Configuration Options

**API URL** (most important):
```
docosaurus/.env.local:
REACT_APP_API_URL=http://your-api.com/api/chat
```

**Welcome Message**:
```
Edit: docosaurus/src/components/ChatWidget/index.tsx
Line 47-53: Change initial messages
```

**Position/Size**:
```
Edit: docosaurus/src/components/ChatWidget/styles.css
.chat-widget-fab: bottom, right (position)
.chat-widget-window: width, height, bottom (size & position)
```

**Colors**:
```
Uses automatic Docosaurus theme colors:
--ifm-color-primary: Main color
--ifm-color-primary-dark: Hover color
[data-theme='dark']: Dark mode colors
```

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Button not visible | Refresh browser (F5), check console (F12) |
| Can't send messages | Backend not running or CORS not configured |
| Empty response | Backend not returning answer field |
| Slow responses | Check backend performance or network latency |
| CORS error | Add http://localhost:3000 to CORS allowed origins in backend |
| TypeScript error | Run `npm install` in docosaurus/ |

---

## Files to Know

| Path | Purpose |
|------|---------|
| `docosaurus/src/components/ChatWidget/` | Main component & styles |
| `docosaurus/src/theme/Root.tsx` | Where widget is mounted |
| `docosaurus/.env.local` | Environment config |
| `docosaurus/FRONTEND_INTEGRATION.md` | Full documentation |

---

## Next Steps

After verifying it works locally:

1. **Deploy Backend**
   - AWS EC2, Google Cloud Run, or your server
   - Update `REACT_APP_API_URL` to production endpoint

2. **Deploy Frontend**
   - Run `npm run build` → static HTML/JS
   - Deploy to Vercel, GitHub Pages, or your host

3. **Monitor**
   - Check latency (target: <3s)
   - Monitor error rates
   - Track user engagement

---

## Support

- 📋 Full guide: `docosaurus/FRONTEND_INTEGRATION.md`
- ✅ Test checklist: `CHATWIDGET_VERIFICATION.md`
- 📝 Implementation record: `history/prompts/textbook-v1/010-*.green.prompt.md`

---

## One-Line Commands

```bash
# Start everything
(cd backend && python -m uvicorn api:app --reload &) && (cd docosaurus && npm run start)

# Just check if services are running
curl http://localhost:8000/health  # Backend
curl http://localhost:3000          # Frontend

# Test API directly
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

---

**Version**: 1.0.0
**Status**: ✅ Production Ready
**Last Updated**: 2025-12-18

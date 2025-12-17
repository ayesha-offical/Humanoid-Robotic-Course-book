# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `1-textbook-platform` | **Date**: 2025-12-06 | **Spec**: `specs/textbook-v1/spec.md`
**Status**: Core RAG Chatbot Implementation

---

## Summary

Build an AI-native, dual-stack platform comprising a Docosaurus-based 13-week Physical AI curriculum and an integrated RAG chatbot (FastAPI + OpenAI Agents SDK + Qdrant). The platform delivers hands-on content for embodied AI (ROS 2, Gazebo, NVIDIA Isaac, sim-to-real transfer) with live Q&A via intelligent agents that ingest curriculum content.

**Technical approach**: Phased rollout (Foundation → Infrastructure + Core Agents → Pilot Content → Full Scaling) with strict reproducibility (Docker, versioned dependencies, CI/CD validation) and performance budgets (chatbot <3s p95 latency, 99.5% uptime).

---

## Technical Context

**Language/Version**: Python 3.10+ (backend), Node.js 18+ (Docosaurus), TypeScript (React components)

**Primary Dependencies**:
- **Agents**: OpenAI Agents SDK (latest), ChatKit SDK (latest, for streaming + context)
- **Backend**: FastAPI 0.104+, Qdrant Python SDK, uvicorn
- **Frontend**: Docosaurus 3.x, MDX, React (TypeScript), TailwindCSS
- **Robotics**: ROS 2 Humble, Gazebo, NVIDIA Isaac Sim (content examples)
- **DevOps**: Docker, Docker Compose, GitHub Actions (deployment)

**Storage**:
- **Qdrant Cloud** (vector embeddings for curriculum)
- **GitHub Pages** (static Docosaurus site, <1 GB total)

**Testing**:
- Backend: pytest (FastAPI API tests)
- Frontend: Docosaurus build validation (0 warnings)
- Content: All code examples run in Docker (Ubuntu 22.04 + ROS 2 Humble)

**Target Platform**: Linux (Ubuntu 22.04 LTS); cloud deployment (AWS/Google Cloud for FastAPI + Qdrant)

**Project Type**: Web application with backend API + static frontend + code examples repository

**Performance Goals**:
- Chatbot query response: p95 ≤3s, average ≤2.5s
- Docosaurus site load: <2s (Lighthouse ≥90)
- Concurrent chatbot requests: ≥50 simultaneous
- Embedding ingestion: batch processing, <$500/month OpenAI cost

**Constraints**:
- Hard deadline: November 30, 2025
- All dependencies pinned (no `*` or `latest` in production)
- 100% reproducibility across 3 machines (Docker + devcontainer enforcement)
- Chatbot uptime: 99.5% SLA
- GitHub Pages storage: ~500 MB (Docosaurus build artifact)

**Scale/Scope**:
- 13-week curriculum = 4 modules + capstone
- ~500 markdown files (~100,000 lines of content)
- ≥3 end-to-end sim-to-real demo videos
- RAG index size: <2 GB (all curriculum vectorized)

---

## Constitution Check

**Constitution v1.0.0 Alignment (Ratified 2025-12-06):**

| Principle | Check | Status | Justification |
|-----------|-------|--------|---------------|
| **I. Hands-On First** | Every lesson includes runnable code examples | ✅ PASS | Spec 1.4.1: Phase 2 content authoring enforces copy-paste-run within 15 min |
| **II. Reproducible Code & Environments** | All dependencies pinned; Docker/devcontainer per lesson | ✅ PASS | Spec Constraint D: Docker per module; GitHub Actions CI validates all code |
| **III. Sim-to-Real Pipeline** | ≥3 modules demonstrate Gazebo → real hardware transfer | ✅ PASS | Spec 1.4.1: ≥3 modules include recorded deployments |
| **IV. RAG-Friendly Structure** | Markdown split by topic (<3000 tokens/file); YAML frontmatter | ✅ PASS | Phase 2 enforces max 3000 tokens/file |
| **V. Quality Code Standards** | PEP 8, Google-style docstrings, type hints on public APIs | ✅ PASS | FastAPI OpenAPI auto-docs; all Python code reviews enforce standards |
| **VI. Practical Prerequisites & Hardware Tiers** | Every lesson lists hardware required | ✅ PASS | Spec Constraint B: Explicit hardware tier labeling |
| **VII. Assessment & Checkpoints** | Measurable goals: latency <3s, deployment testable | ✅ PASS | Spec 2.2-2.4: Chatbot <3s p95 latency |

**All gates PASS.** Implementation plan is constitutionally aligned.

---

## Project Structure

### Frontend Components

```
docosaurus/src/components/
├── ChatbotWidget.tsx          # Chat widget UI
├── HomepageFeatures/          # Homepage content
├── LanguageSwitcher.tsx       # Language selection
└── ...
```

### Backend Services

```
backend/src/
├── api/
│   ├── chat.py                # /api/ask endpoint
│   ├── health.py              # /api/health
│   └── admin.py               # /api/admin/*
│
├── agents/
│   ├── rag_agent.py           # OpenAI Agents SDK integration
│   └── retrievers.py          # Qdrant retrieval
│
├── services/
│   ├── embedding_ingest.py    # Batch ingest + metadata
│   ├── llm.py                 # OpenAI + ChatKit wrapper
│   └── cache.py               # Redis cache
│
└── models/
    ├── schemas.py             # Pydantic schemas
    └── db.py                  # SQLAlchemy models (if needed)
```

---

## Phased Implementation Roadmap

### Phase 0: Foundation & Research (Weeks 1-2, by Dec 13)

**Objectives**: Research + setup + provision cloud services

**Deliverables**:
- ✅ GitHub repository + CI/CD scaffolding
- ✅ Docker images (ROS 2 + FastAPI)
- ✅ Qdrant Cloud provisioned
- ✅ OpenAI Agents SDK pinned

---

### Phase 1: Infrastructure & Contracts (Weeks 2-3, by Dec 20)

**Objectives**: Data models, API contracts, deployment pipeline

**Deliverables**:
- ✅ `data-model.md` (entities, relationships)
- ✅ `contracts/chatbot-api.yaml` (OpenAPI specs)
- ✅ `quickstart.md` (dev setup)
- ✅ Backend skeleton (FastAPI)
- ✅ Docosaurus skeleton
- ✅ GitHub Actions workflows

---

### Phase 2: Pilot Content & Core Agents (Weeks 3-4, by Dec 27)

**Objectives**: Module 1 + RAG pipeline

**Deliverables**:
- ✅ Module 1 (ROS 2): 10-15 lessons, code examples, 1 demo video
- ✅ Embedding ingestion pipeline (chunks + OpenAI embeddings + Qdrant)
- ✅ RAG agent (OpenAI Agents SDK + ChatKit SDK)
- ✅ Chatbot API: `/api/ask`
- ✅ Latency tests: 100 queries, p95 ≤3s

---

### Phase 3: Full Content & Hardening (Weeks 4-5, by Nov 23)

**Objectives**: Modules 2-4 + Capstone + optimization + pre-launch

**Deliverables**:
- ✅ Modules 2, 3, 4 + Capstone (40+ lessons total)
- ✅ All code examples tested in Docker + GitHub Actions CI
- ✅ Chatbot latency optimization: p95 ≤3s over 1,000 queries
- ✅ Uptime test: 99.5% SLA over 7 days
- ✅ Load test: ≥50 concurrent queries
- ✅ Documentation complete (README, CONTRIBUTING, troubleshooting)
- ✅ Pre-launch checklist (14+ items)

---

### Phase 4: Launch & Post-Launch (Week 5+, by Nov 30)

**Objectives**: Production deployment + monitoring + community

**Deliverables**:
- ✅ Docosaurus site live on GitHub Pages
- ✅ FastAPI backend deployed (AWS EC2 or Google Cloud Run)
- ✅ Qdrant Cloud scaled for production
- ✅ Monitoring dashboard (latency, uptime, costs)
- ✅ Community channels active (GitHub Issues, Discussions)
- ✅ ≥100 GitHub stars tracked

---

## Key Design Decisions

### D1: Technology Stack

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **Chat Streaming** | ChatKit SDK | Real-time streams, works with FastAPI WebSockets |
| **RAG Retrieval** | Qdrant + OpenAI embeddings | Managed, scalable, accurate |
| **Backend** | FastAPI | Async, OpenAPI auto-docs, Python ecosystem |

### D2: RAG Architecture

**Content Ingestion**:
- Chunk Docosaurus markdown by topic (<3000 tokens/file)
- Embed chunks with OpenAI embeddings
- Store vectors + metadata in Qdrant

**Query Processing**:
- Accept natural language query
- Retrieve top-5 relevant chunks from Qdrant
- Pass to OpenAI Agents SDK with context
- Stream response via ChatKit SDK

---

## API Contracts

### Chat Endpoints

```yaml
POST /api/ask
Request:
  query: string
  context?: string (optional)
Response (200):
  answer: string
  sources: array[{title: string, url: string}]
  confidence: number (0-1)
  latency_ms: integer

GET /api/health
Response (200):
  status: string ("healthy"|"degraded"|"unavailable")
  uptime_seconds: integer
```

---

## Risks & Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| OpenAI Agents SDK breaking changes | Medium | Pin version; test monthly; fallback to Chat Completions API |
| Qdrant cost spike | Medium | Rate limiting (100 req/min per user); cost dashboard |
| GitHub Pages storage limit | Low | Compress assets; YouTube/Vimeo for videos |
| Modules not complete by Nov 23 | High | Prioritize Modules 1-2; reduce Modules 3-4 scope if needed |
| Chatbot latency >3s | High | Implement request caching; optimize Qdrant queries |

---

## Success Criteria (Measurable)

By November 30, 2025, **ALL** of the following must be met:

- ✅ Docosaurus site live (<2s load, Lighthouse ≥90)
- ✅ FastAPI `/api/ask` operational
- ✅ Qdrant >90% curriculum coverage
- ✅ Chatbot latency: p95 ≤3s (1,000 queries)
- ✅ Chatbot accuracy: 90% FAQ correct
- ✅ 100% code examples runnable in Docker
- ✅ ≥3 sim-to-real demo videos
- ✅ Uptime: 99.5% (7-day test)
- ✅ No TODOs/FIXMEs in released content
- ✅ README + CONTRIBUTING finalized
- ✅ ≥100 GitHub stars

---

## Revision History

| Version | Date | Author | Change |
|---------|------|--------|--------|
| 1.0 | 2025-12-06 | AI Architect | Initial plan; core RAG implementation without auth/database |

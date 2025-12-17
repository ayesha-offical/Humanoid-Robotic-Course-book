# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform (v2 - Hackathon Bonus)

**Branch**: `1-textbook-platform` | **Date**: 2025-12-06 | **Spec**: `specs/textbook-v1/spec.md`
**Status**: Updated with Hackathon Bonus Features (150 pts)

---

## Summary

Build an AI-native, dual-stack platform comprising a Docosaurus-based 13-week Physical AI curriculum and an integrated RAG chatbot (FastAPI + OpenAI Agents SDK + ChatKit SDK + Qdrant). The platform delivers hands-on content for embodied AI (ROS 2, Gazebo, NVIDIA Isaac, sim-to-real transfer) with **personalization and multi-language support (Urdu)**, enabling live Q&A via intelligent agents that ingest curriculum content.

**🏆 Hackathon Bonus Features (150 pts):**
1. **Mandatory Better-Auth Authentication** (50 pts): Signup captures user `hardware_background` + `software_background`; enables personalized learning paths.
2. **Personalization Engine** (50 pts): Dynamically rewrites explanations based on user expertise level (beginner/intermediate/advanced).
3. **Urdu Translation Agent** (50 pts): Async translation of curriculum chapters; sidebar widget for Urdu learners.

**Technical approach**: Phased rollout (Foundation → Infrastructure + Auth → Pilot Content + Agents → Full Scaling + Personalization) with strict reproducibility (Docker, versioned dependencies, CI/CD validation) and performance budgets (chatbot <3s p95 latency, 99.5% uptime, personalization <500ms overhead).

---

## Technical Context

**Language/Version**: Python 3.10+ (backend), Node.js 18+ (Docosaurus), TypeScript (React components)

**Primary Dependencies**:
- **Authentication**: Better-Auth 0.15+ (user signup/login with background capture) ← BONUS
- **Agents**: OpenAI Agents SDK (latest), ChatKit SDK (latest, for streaming + context)
- **Backend**: FastAPI 0.104+, Qdrant Python SDK, uvicorn
- **Translation**: Google Translate API (Urdu + 20+ languages) ← BONUS
- **Frontend**: Docusaurus 3.x, MDX, React (TypeScript), TailwindCSS
- **Robotics**: ROS 2 Humble, Gazebo, NVIDIA Isaac Sim (content examples)
- **DevOps**: Docker, Docker Compose, GitHub Actions (deployment)

**Storage**:
- **Neon PostgreSQL** (users, user_preferences, chat_sessions, deployment_logs):
  - `users`: id, email, password_hash, hardware_background, software_background, created_at, updated_at ← BONUS
  - `user_preferences`: user_id, personalization_enabled, language, theme, updated_at ← BONUS
  - `chat_sessions`: id, user_id, query_count, context_tokens, created_at, messages JSONB
  - `translations`: content_id, language, translated_text, created_at ← BONUS
- **Qdrant Cloud** (vector embeddings for curriculum, supports filtering by hardware_tier + user context)
- **GitHub Pages** (static Docosaurus site, <1 GB total)

**Testing**:
- Backend: pytest (FastAPI API tests, personalization accuracy, translation quality)
- Frontend: Docosaurus build validation (0 warnings)
- Content: All code examples run in Docker (Ubuntu 22.04 + ROS 2 Humble)

**Target Platform**: Linux (Ubuntu 22.04 LTS); cloud deployment (AWS/Google Cloud for FastAPI + Qdrant)

**Project Type**: Web application with backend API + static frontend + code examples repository

**Performance Goals**:
- Chatbot query response: p95 ≤3s, average ≤2.5s
- **Personalization latency: <500ms overhead** (user context lookup + prompt rewrite) ← BONUS
- **Urdu translation latency: <1s per chapter** (async batch) ← BONUS
- Docosaurus site load: <2s (Lighthouse ≥90)
- Concurrent chatbot requests: ≥50 simultaneous
- **Concurrent personalized queries: ≥20 simultaneous** ← BONUS
- Embedding ingestion: batch processing, <$500/month OpenAI cost
- **Better-Auth signup/login: <500ms end-to-end** ← BONUS

**Constraints**:
- Hard deadline: November 30, 2025 (130 base pts + 150 bonus pts = 280 total)
- All dependencies pinned (no `*` or `latest` in production)
- 100% reproducibility across 3 machines (Docker + devcontainer enforcement)
- **Authentication mandatory** (Better-Auth, no GitHub OAuth fallback for v1.0) ← BONUS
- Chatbot uptime: 99.5% SLA
- **Personalization & translation services uptime: 99% SLA** (async, non-blocking) ← BONUS
- GitHub Pages storage: ~500 MB (Docosaurus build artifact)
- User data privacy: No PII exposed; hashed passwords via Better-Auth ← BONUS

**Scale/Scope**:
- 13-week curriculum = 4 modules + capstone
- ~500 markdown files (~100,000 lines of content)
- ≥3 end-to-end sim-to-real demo videos
- RAG index size: <2 GB (all curriculum vectorized)
- **User base**: Support ≥500 concurrent users with personalization ← BONUS

---

## Constitution Check

**Constitution v1.0.0 Alignment (Ratified 2025-12-06):**

| Principle | Check | Status | Justification |
|-----------|-------|--------|---------------|
| **I. Hands-On First** | Every lesson includes runnable code examples | ✅ PASS | Spec 1.4.1: Phase 2 content authoring enforces copy-paste-run within 15 min |
| **II. Reproducible Code & Environments** | All dependencies pinned; Docker/devcontainer per lesson | ✅ PASS | Spec Constraint D: Docker per module; GitHub Actions CI validates all code |
| **III. Sim-to-Real Pipeline** | ≥3 modules demonstrate Gazebo → real hardware transfer | ✅ PASS | Spec 1.4.1: ≥3 modules include recorded deployments |
| **IV. RAG-Friendly Structure** | Markdown split by topic (<3000 tokens/file); YAML frontmatter | ✅ PASS | Phase 2 enforces max 3000 tokens/file; personalization metadata support ← BONUS |
| **V. Quality Code Standards** | PEP 8, Google-style docstrings, type hints on public APIs | ✅ PASS | FastAPI OpenAPI auto-docs; all Python code reviews enforce standards |
| **VI. Practical Prerequisites & Hardware Tiers** | Every lesson lists hardware required | ✅ PASS | Spec Constraint B: Explicit hardware tier labeling; user background captured ← BONUS |
| **VII. Assessment & Checkpoints** | Measurable goals: latency <3s, deployment testable | ✅ PASS | Spec 2.2-2.4: Chatbot <3s p95 latency; personalization <500ms overhead ← BONUS |

**All gates PASS.** Implementation plan is constitutionally aligned and bonus-feature-ready.

---

## Project Structure (Updated for Bonus Features)

### Frontend Components (BONUS updates)

```
docusaurus/src/components/
├── AuthModal.tsx              # Better-Auth signup/login (BONUS)
│   ├── hardware_background    # Dropdown: "Beginner", "Intermediate", "Advanced"
│   └── software_background    # Dropdown: "No coding", "Some Python", "Advanced ML"
├── PersonalizeButton.tsx      # Toggle personalization mode (BONUS)
├── LanguageSwitcher.tsx       # Language selection (English, Urdu, Spanish, etc.) (BONUS)
├── UrduTranslationPanel.tsx   # Sidebar showing Urdu translations (BONUS)
├── ChatbotWidget.tsx          # Updated with personalization context
└── ProfilePage.tsx            # User preferences (language, personalization toggle) (BONUS)
```

### Backend Services (BONUS updates)

```
backend/src/
├── api/
│   ├── auth.py                # Better-Auth endpoints (BONUS)
│   │   ├── POST /api/auth/signup
│   │   └── POST /api/auth/login
│   ├── chat.py                # /api/ask (now with user_context param)
│   ├── personalization.py     # /api/personalize/* (BONUS)
│   │   ├── POST /api/personalize/explain
│   │   └── GET /api/personalize/user-profile
│   ├── translation.py         # /api/translate/* (BONUS)
│   │   ├── POST /api/translate/chapter
│   │   └── GET /api/translate/chapter-list
│   └── admin.py               # /api/admin/* (user analytics, cost tracking)
│
├── agents/
│   ├── rag_agent.py           # OpenAI Agents SDK integration
│   ├── personalization_agent.py # Dynamic prompt rewriting (BONUS)
│   │   ├── rewrite_for_beginner()
│   │   ├── rewrite_for_intermediate()
│   │   └── rewrite_for_advanced()
│   ├── urdu_translator.py     # Google Translate wrapper (BONUS)
│   └── retrievers.py          # Qdrant retrieval + user context filtering
│
├── services/
│   ├── auth_service.py        # Better-Auth integration (BONUS)
│   ├── personalization_service.py # User background lookup (BONUS)
│   ├── translation_service.py # Async Urdu translation (BONUS)
│   ├── embedding_ingest.py    # Batch ingest + metadata
│   ├── llm.py                 # OpenAI + ChatKit wrapper
│   ├── database.py            # Neon PostgreSQL ORM
│   └── cache.py               # Redis cache (translations, user profiles)
│
└── models/
    ├── schemas.py             # Updated Pydantic schemas (BONUS fields)
    └── db.py                  # Updated SQLAlchemy models (BONUS tables)
```

### Database Schema Updates (BONUS)

```sql
-- Users table (BONUS)
CREATE TABLE users (
  id UUID PRIMARY KEY,
  email VARCHAR UNIQUE NOT NULL,
  password_hash VARCHAR NOT NULL,
  hardware_background ENUM('beginner', 'intermediate', 'advanced'),
  software_background ENUM('none', 'some_python', 'advanced_ml'),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- User preferences (BONUS)
CREATE TABLE user_preferences (
  user_id UUID PRIMARY KEY REFERENCES users(id),
  personalization_enabled BOOLEAN DEFAULT true,
  language VARCHAR DEFAULT 'en',
  theme ENUM('light', 'dark') DEFAULT 'light',
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Translations cache (BONUS)
CREATE TABLE translations (
  id UUID PRIMARY KEY,
  content_id VARCHAR,
  language VARCHAR,
  translated_text TEXT,
  created_at TIMESTAMP DEFAULT NOW(),
  UNIQUE(content_id, language)
);

-- Chat sessions (updated)
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id),
  query_count INTEGER,
  created_at TIMESTAMP,
  messages JSONB
);
```

---

## Phased Implementation Roadmap (BONUS-Enhanced)

### Phase 0: Foundation & Research (Weeks 1-2, by Dec 13)

**Objectives**: Research + setup + provision cloud services

**Deliverables**:
- ✅ Better-Auth API documentation reviewed + Hello World implementation ← BONUS
- ✅ Google Translate API pricing + quota testing ← BONUS
- ✅ ChatKit SDK integration tested (streaming chat) ← BONUS
- ✅ GitHub repository + CI/CD scaffolding
- ✅ Docker images (ROS 2 + FastAPI)
- ✅ Neon PostgreSQL + Qdrant Cloud provisioned
- ✅ OpenAI Agents SDK pinned

**Research Unknowns**:
1. **Better-Auth with custom fields** (hardware_background, software_background) ← BONUS
2. **Google Translate API batch cost** (estimated 5K translations/month) ← BONUS
3. **ChatKit SDK streaming compatibility** with FastAPI WebSockets ← BONUS

---

### Phase 1: Infrastructure, Auth & Contracts (Weeks 2-3, by Dec 20)

**Objectives**: Data models, API contracts, **Better-Auth setup** (50 pts BONUS), deployment pipeline

**Deliverables - BONUS Features**:
- ✅ **Better-Auth Integration** (50 pts):
  - Better-Auth library 0.15+ installed + configured
  - `/api/auth/signup` endpoint with hardware_background + software_background capture
  - `/api/auth/login` endpoint with secure session management
  - Better-Auth secrets in `.env` (no hardcoded values)
  - AuthModal React component (frontend)
  - Database: `users` + `user_preferences` tables deployed
- ✅ **Personalization Schema** (supports 50 pts feature):
  - `user_preferences` table with personalization_enabled + language fields
  - API contract: `POST /api/personalize/explain` (accepts user_context)
  - PersonalizeButton component (UI toggle)
- ✅ **Translation Schema** (supports 50 pts feature):
  - `translations` cache table in PostgreSQL
  - API contract: `POST /api/translate/chapter` (language + content_id)
  - Google Translate API client initialized
  - LanguageSwitcher component (UI dropdown)

**Deliverables - Base Features**:
- ✅ `data-model.md` (entities, relationships, BONUS DB schema)
- ✅ `contracts/chatbot-api.yaml` (OpenAPI specs for auth + personalization + translation)
- ✅ `quickstart.md` (dev setup with Better-Auth)
- ✅ Backend skeleton (FastAPI with auth endpoints)
- ✅ Docosaurus skeleton (Auth + Personalize + Language components)
- ✅ GitHub Actions workflows (auth tests, build validation)

**Dependencies**:
- Phase 0 complete (research, repos, cloud services)
- Better-Auth 0.15+ tested (Phase 0)
- Google Translate API key provisioned (Phase 0)

---

### Phase 2: Pilot Content & Core Agents (Weeks 3-4, by Dec 27)

**Objectives**: Module 1 + RAG pipeline + **personalization engine** (50 pts) + **Urdu translation** (50 pts)

**Deliverables - BONUS Features**:
- ✅ **Personalization Engine** (50 pts):
  - `src/services/personalization_service.py`: Lookup user hardware_background + software_background
  - `src/agents/personalization_agent.py`: Prompt rewriting based on level
    - Beginner: Add fundamentals, step-by-step, simple language
    - Intermediate: Skip basics, focus on application
    - Advanced: Research papers, optimization, tradeoffs
  - `POST /api/personalize/explain`: Accept query + user context → rewritten answer
  - PersonalizeButton toggle: Enable/disable per query
  - **Tests**: 10 queries per level, verify explanation clarity
  - **Latency**: <500ms overhead (user profile cached)

- ✅ **Urdu Translation Service** (50 pts):
  - `src/services/translation_service.py`: Google Translate wrapper
  - `src/agents/urdu_translator.py`: Async batch translation
  - `POST /api/translate/chapter`: Accept chapter_id + language → translated text
  - LanguageSwitcher: Allow language selection at lesson top
  - UrduTranslationPanel: Sidebar showing Urdu key terms
  - PostgreSQL caching: Store translations (content_id, language, text)
  - **Tests**: Translate 5 chapters to Urdu, verify quality
  - **Latency**: <1s per chapter (async, non-blocking)

**Deliverables - Base Features**:
- ✅ Module 1 (ROS 2): 10-15 lessons, code examples, 1 demo video
- ✅ Embedding ingestion pipeline (chunks + OpenAI embeddings + Qdrant)
- ✅ RAG agent (OpenAI Agents SDK + ChatKit SDK)
- ✅ Chatbot API:
  - `/api/ask`: Standard queries (no auth required)
  - `/api/ask?personalize=true`: Personalized (requires auth) ← BONUS
  - `/api/translate/ask`: Multilingual queries ← BONUS
- ✅ Chatbot widget (AuthModal + PersonalizeButton + LanguageSwitcher)
- ✅ Latency tests:
  - Standard: 100 queries, p95 ≤3s
  - Personalized: 50 queries, p95 ≤3.5s (overhead <500ms) ← BONUS
  - Translated: 30 queries, p95 ≤4s (async) ← BONUS

**Dependencies**:
- Phase 1 complete (Better-Auth, personalization schema, translation schema)
- Google Translate API tested (Phase 0)
- ChatKit SDK integration done (Phase 0)

---

### Phase 3: Full Content & Hardening (Weeks 4-5, by Nov 23)

**Objectives**: Modules 2-4 + Capstone + optimization + pre-launch

**Deliverables - BONUS-Aware**:
- ✅ Personalization accuracy test: Verify all 3 levels (beginner/intermediate/advanced) produce contextually correct answers
- ✅ Urdu translation quality check: Native speaker review of 5 chapters
- ✅ Better-Auth session security: Test logout, session expiry, token refresh
- ✅ Concurrent personalized load test: ≥20 simultaneous personalized queries
- ✅ Concurrent translation load test: ≥20 simultaneous translation requests

**Deliverables - Base Features**:
- ✅ Modules 2, 3, 4 + Capstone (40+ lessons total)
- ✅ All code examples tested in Docker + GitHub Actions CI
- ✅ Chatbot latency optimization: p95 ≤3s over 1,000 queries
- ✅ Uptime test: 99.5% SLA over 7 days
- ✅ Load test: ≥50 concurrent standard queries
- ✅ Documentation complete (README, CONTRIBUTING, troubleshooting)
- ✅ Pre-launch checklist (14+ items)

**Dependencies**:
- Phase 2 complete (personalization + translation agents working)

---

### Phase 4: Launch & Post-Launch (Week 5+, by Nov 30)

**Objectives**: Production deployment + monitoring + community

**Deliverables**:
- ✅ Docosaurus site live on GitHub Pages
- ✅ FastAPI backend deployed (AWS EC2 or Google Cloud Run)
- ✅ Qdrant Cloud scaled for production
- ✅ **Better-Auth production config** (password hashing, session tokens) ← BONUS
- ✅ **Translation cache warmed** (popular chapters pre-translated to Urdu) ← BONUS
- ✅ Monitoring dashboard (latency, uptime, costs, **personalization accuracy**, **translation quality**)
- ✅ Community channels active (GitHub Issues, Discussions)
- ✅ ≥100 GitHub stars tracked

---

## Key Design Decisions (BONUS-Updated)

### D1: Technology Stack

| Component | Choice | Rationale | BONUS Impact |
|-----------|--------|-----------|--------------|
| **Authentication** | Better-Auth | Secure, minimal setup, custom field support | Enables hardware/software background capture → personalization |
| **Personalization** | Dynamic prompt rewriting | Simple, fast, no model fine-tuning required | Can run <500ms per query |
| **Translation** | Google Translate API | Accurate, 100+ languages, cached results | Urdu support ready, async non-blocking |
| **Chat Streaming** | ChatKit SDK | Real-time streams, works with FastAPI WebSockets | Personalized + translated responses stream smoothly |

### D2: Personalization Strategy (BONUS)

**User Backgrounds** (captured at signup):
- `hardware_background`: "Beginner" / "Intermediate" / "Advanced"
- `software_background`: "No coding" / "Some Python" / "Advanced ML"

**Prompt Rewriting Rules**:
- **Beginner + No Coding**: Add definitions, step-by-step code, analogies to real world
- **Beginner + Some Python**: Assume Python basics, focus on robotics concepts
- **Intermediate + Advanced ML**: Skip theory, focus on implementation tricks + optimization
- **Advanced**: Include research citations, performance budgets, tradeoffs

**Latency Optimization**:
- Cache user profile in Redis (hardware_background, software_background)
- Cached lookup: <50ms
- Prompt rewriting: <200ms
- LLM inference: <250ms
- **Total: <500ms overhead** ✅

### D3: Urdu Translation Strategy (BONUS)

**Scope**:
- Translate lesson headers + summaries (not full body text, to control costs)
- Cache translations in PostgreSQL (avoid re-translating)
- Async batch: Don't block chatbot queries

**Latency**:
- First-time translation of chapter: <1s (async, user doesn't wait)
- Cached translation retrieval: <50ms (cached in PostgreSQL + Redis)

**Cost Control**:
- Estimated 5K translations/month (500 lessons × 10 languages)
- Google Translate API: ~$1/million characters → <$2/month
- ✅ Negligible cost

---

## API Contracts (BONUS)

### Authentication Endpoints

```yaml
POST /api/auth/signup
Request:
  email: string
  password: string
  hardware_background: enum (beginner|intermediate|advanced)
  software_background: enum (none|some_python|advanced_ml)
Response (201):
  user_id: string
  session_token: string

POST /api/auth/login
Request:
  email: string
  password: string
Response (200):
  user_id: string
  session_token: string
```

### Personalization Endpoints (BONUS)

```yaml
POST /api/personalize/explain
Request:
  query: string
  user_id: string (from auth)
Response (200):
  answer: string (personalized based on hardware_background + software_background)
  personalization_level: enum (beginner|intermediate|advanced)
  latency_ms: integer
```

### Translation Endpoints (BONUS)

```yaml
POST /api/translate/chapter
Request:
  chapter_id: string
  language: string (e.g., "ur" for Urdu, "es" for Spanish)
Response (200):
  chapter_id: string
  language: string
  translated_title: string
  translated_summary: string
  cached: boolean (true if from PostgreSQL, false if freshly translated)

GET /api/translate/supported-languages
Response (200):
  languages: array[{code: string, name: string}]
```

---

## Risks & Mitigations (BONUS-Updated)

| Risk | Severity | Mitigation |
|------|----------|-----------|
| **Better-Auth compatibility with FastAPI** | Medium | Test in Phase 0; have fallback JWT implementation ready |
| **Personalization accuracy <80%** (BONUS) | Medium | Curate prompt templates per level; test with 30 queries per level before Phase 3 |
| **Urdu translation quality poor** (BONUS) | Medium | Use native speaker review; fall back to English if score <0.8 |
| **Concurrent personalized queries slow down (<500ms)** (BONUS) | Medium | Implement Redis caching for user profiles; scale to 2+ FastAPI replicas |
| **Translation API quota exceeded** (BONUS) | Low | Batch offline at night; limit language choices to 10 most requested |
| OpenAI Agents SDK breaking changes | Medium | Pin version; test monthly; fallback to Chat Completions API |
| Qdrant cost spike | Medium | Rate limiting (100 req/min per user); cost dashboard |
| GitHub Pages storage limit | Low | Compress assets; YouTube/Vimeo for videos |
| Modules not complete by Nov 23 | High | Prioritize Modules 1-2; reduce Modules 3-4 scope if needed |

---

## Success Criteria (Measurable - BONUS-Enhanced)

By November 30, 2025, **ALL** of the following must be met:

**Base Features** (130 pts):
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

**BONUS Features** (150 pts):
- ✅ **Better-Auth (50 pts)**:
  - `/api/auth/signup` accepts hardware_background + software_background
  - `/api/auth/login` works with secure sessions
  - AuthModal component displays + functions
  - User profile persisted in Neon
  - Latency: <500ms signup/login

- ✅ **Personalization (50 pts)**:
  - `/api/personalize/explain` returns contextually correct answers
  - PersonalizeButton toggle works in UI
  - Beginner/Intermediate/Advanced levels verified (10 queries each)
  - Latency: <500ms overhead
  - Concurrent: ≥20 simultaneous personalized queries

- ✅ **Urdu Translation (50 pts)**:
  - `/api/translate/chapter` returns Urdu translations
  - LanguageSwitcher component allows language selection
  - UrduTranslationPanel displays translations
  - 5 chapters translated + verified for quality
  - Latency: <1s per chapter
  - Concurrent: ≥20 simultaneous translation requests

---

## Revision History

| Version | Date | Author | Change |
|---------|------|--------|--------|
| 1.0 | 2025-12-06 | AI Architect | Initial plan; 4-phase roadmap, architecture overview |
| 2.0 (Bonus) | 2025-12-06 | AI Architect | Integrated 150 pts hackathon bonus features: Better-Auth + Personalization + Urdu Translation |


# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform

**Branch**: `1-textbook-platform` | **Date**: 2025-12-06 | **Spec**: `specs/textbook-v1/spec.md`
**Input**: Feature specification from `specs/textbook-v1/spec.md`

---

## Summary

Build an AI-native, dual-stack platform comprising a Docusaurus-based 13-week Physical AI curriculum and an integrated RAG chatbot (FastAPI + OpenAI Agents SDK + ChatKit SDK + Qdrant). The platform delivers hands-on content for embodied AI (ROS 2, Gazebo, NVIDIA Isaac, sim-to-real transfer) with personalization and multi-language support (Urdu), enabling live Q&A via intelligent agents that ingest curriculum content. **Hackathon Bonus Features (150 pts)**: Mandatory Better-Auth authentication (50 pts) capturing user hardware/software backgrounds; personalization engine that dynamically rewrites explanations based on user context (50 pts); Urdu translation agent for accessibility (50 pts). Technical approach: phased rollout (Foundation → Infrastructure + Auth → Pilot Content + Agents → Full Scaling + Personalization) with strict reproducibility (Docker, versioned dependencies, CI/CD validation) and performance budgets (chatbot <3s p95 latency, 99.5% uptime, personalization <500ms latency overhead).

---

## Technical Context

**Language/Version**: Python 3.10+ (backend), Node.js 18+ (Docusaurus), TypeScript (React components)
**Primary Dependencies**:
  - **Authentication**: Better-Auth 0.15+ (user signup/login with background capture)
  - **Agents**: OpenAI Agents SDK (latest), ChatKit SDK (latest, for streaming + context)
  - **Backend**: FastAPI 0.104+, Qdrant Python SDK, uvicorn
  - **Translation**: Google Translate API (Urdu + 20+ languages)
  - **Frontend**: Docusaurus 3.x, MDX, React (TypeScript), TailwindCSS
  - **Robotics**: ROS 2 Humble, Gazebo, NVIDIA Isaac Sim (content examples)
  - **DevOps**: Docker, Docker Compose, GitHub Actions (deployment)

**Storage**:
  - **Neon PostgreSQL** (users, user_preferences, chat_sessions, deployment_logs)
    - `users` table: id, github_id, hardware_background, software_background, created_at, updated_at
    - `user_preferences` table: user_id, personalization_enabled, language, theme, created_at
    - `chat_sessions` table: id, user_id, query_count, context_tokens, created_at, messages JSONB
  - **Qdrant Cloud** (vector embeddings for curriculum content, supports filtering by hardware_tier)
  - **GitHub Pages** (static Docusaurus site, <1 GB total)

**Testing**:
  - Backend: pytest (FastAPI API tests, chatbot accuracy tests)
  - Frontend: Docusaurus build validation (0 warnings)
  - Content: All code examples run in Docker (Ubuntu 22.04 + ROS 2 Humble)

**Target Platform**: Linux (Ubuntu 22.04 LTS for dev/test/prod); cloud deployment (AWS/Google Cloud for FastAPI + Qdrant)

**Project Type**: Web application with backend API + static frontend documentation + code examples repository

**Performance Goals**:
  - Chatbot query response: p95 ≤3s, average ≤2.5s
  - Personalization latency: <500ms overhead (user context lookup + prompt rewrite)
  - Urdu translation latency: <1s per chapter (async batch)
  - Docusaurus site load: <2s (Lighthouse ≥90)
  - Concurrent chatbot requests: ≥50 simultaneous without degradation
  - Concurrent personalized queries: ≥20 simultaneous without degradation
  - Embedding ingestion: batch processing, <$500/month OpenAI cost
  - Better-Auth signup/login: <500ms end-to-end

**Constraints**:
  - Hard deadline: November 30, 2025 (130 base pts + 150 bonus pts = 280 total)
  - All dependencies pinned (no `*` or `latest` in production)
  - 100% reproducibility across 3 machines (Docker + devcontainer enforcement)
  - Authentication mandatory (Better-Auth, no GitHub OAuth fallback for v1.0)
  - Chatbot uptime: 99.5% SLA
  - Personalization & translation services uptime: 99% SLA (async, non-blocking)
  - GitHub Pages storage: ~500 MB (Docusaurus build artifact)
  - User data privacy: No PII exposed; hashed passwords via Better-Auth

**Scale/Scope**:
  - 13-week curriculum = 4 modules + capstone
  - ~500 markdown files (~100,000 lines of content)
  - ≥3 end-to-end sim-to-real demo videos
  - RAG index size: <2 GB (all curriculum vectorized)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution v1.0.0 Alignment (Ratified 2025-12-06):**

| Principle | Check | Status | Justification |
|-----------|-------|--------|---------------|
| **I. Hands-On First** | Every lesson includes runnable code examples | ✅ PASS | Spec 1.4.1: "100% of code snippets execute without modification"; Phase 2 content authoring enforces copy-paste-run within 15 min |
| **II. Reproducible Code & Environments** | All dependencies pinned; Docker/devcontainer per lesson | ✅ PASS | Spec Constraint D: "All dependencies pinned"; Docker images per module; GitHub Actions CI validates all code |
| **III. Sim-to-Real Pipeline** | ≥3 modules demonstrate Gazebo → real hardware transfer | ✅ PASS | Spec 1.4.1: "≥3 modules include recorded deployments (Gazebo → Jetson Orin or manipulator)" |
| **IV. RAG-Friendly Structure** | Markdown split by topic (<3000 tokens/file); YAML frontmatter with metadata | ✅ PASS | Phase 2 content structure enforces max 3000 tokens/file; YAML frontmatter mandatory per constitution VII |
| **V. Quality Code Standards** | PEP 8, Google-style docstrings, type hints on public APIs | ✅ PASS | Backend API (FastAPI) includes OpenAPI auto-docs; all Python code reviews enforce PEP 8 + type hints |
| **VI. Practical Prerequisites & Hardware Tiers** | Every lesson lists hardware required (Sim-Only / CPU / GPU RTX / Jetson) | ✅ PASS | Spec Constraint B: Explicit hardware tier labeling; YAML frontmatter includes `hardware_tier` field |
| **VII. Assessment & Checkpoints** | Measurable goals: latency <100ms, deployment w/o manual tuning | ✅ PASS | Spec 2.2-2.4: Chatbot <3s p95 latency (measured), acceptance gates include runnable tests |

**All gates PASS.** Implementation plan is constitutionally aligned.

---

## Project Structure

### Documentation (this feature)

```text
specs/textbook-v1/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (architecture + phased approach)
├── research.md          # Phase 0 output (resolved unknowns)
├── data-model.md        # Phase 1 output (entities, APIs, DB schema)
├── contracts/           # Phase 1 output (OpenAPI specs, data models)
│   ├── chatbot-api.yaml
│   └── embedding-schema.json
├── quickstart.md        # Phase 1 output (setup guide)
└── tasks.md             # Phase 2 output (dependency-ordered tasks)
```

### Source Code (repository root)

```text
# Web application: Frontend (Docusaurus) + Backend (FastAPI + Agents) + Examples (ROS 2)

docusaurus/
├── src/
│   ├── components/
│   │   ├── ChatbotWidget.tsx      # Embedded chat UI with personalization context
│   │   ├── AuthModal.tsx          # Better-Auth signup/login (hardware/software background)
│   │   ├── PersonalizeButton.tsx  # Toggle personalization mode (adjusts explanations)
│   │   ├── LanguageSwitcher.tsx   # Language selection (Urdu + English + others)
│   │   ├── CodeBlock.tsx          # Syntax highlighting
│   │   └── UrduTranslationPanel.tsx # Sidebar panel for Urdu chapter translations
│   ├── pages/
│   │   ├── auth/
│   │   │   ├── signup.tsx
│   │   │   └── login.tsx
│   │   ├── profile.tsx            # User preferences (personalization, language)
│   │   └── dashboard.tsx
│   ├── css/
│   ├── hooks/
│   │   ├── useAuth.ts             # Better-Auth context hook
│   │   ├── usePersonalization.ts  # User background + preferences
│   │   └── useTranslation.ts      # Language selection
│   └── theme/
├── docs/
│   ├── module-1/                  # ROS 2 Foundations
│   │   ├── lesson-1-setup.md
│   │   ├── lesson-2-launch.md
│   │   └── README.md
│   ├── module-2/                  # Control Systems
│   ├── module-3/                  # Embodied Perception
│   ├── module-4/                  # Sim-to-Real Transfer
│   └── capstone/
├── docusaurus.config.js           # Site config + sidebar auto-generation
├── package.json
└── Dockerfile

backend/
├── src/
│   ├── api/
│   │   ├── auth.py                # Better-Auth endpoints (signup, login, logout, profile)
│   │   ├── chat.py                # /api/ask endpoint (now with user_context param)
│   │   ├── personalization.py     # /api/personalize/* (get/update user preferences)
│   │   ├── translation.py         # /api/translate/* (Urdu + multi-language endpoints)
│   │   ├── admin.py               # /api/admin/* (user progress, cost tracking, analytics)
│   │   └── health.py              # /health (uptime monitoring)
│   ├── agents/
│   │   ├── rag_agent.py           # OpenAI Agents SDK integration
│   │   ├── personalization_agent.py # Urdu translation + content personalization (ChatKit SDK)
│   │   ├── retrievers.py          # Qdrant vector retrieval with user context filtering
│   │   ├── prompt_templates.py    # Prompts for standard + personalized + translated queries
│   │   └── urdu_translator.py     # Google Translate API wrapper for Urdu
│   ├── services/
│   │   ├── auth_service.py        # Better-Auth integration + user background capture
│   │   ├── embedding_ingest.py    # Batch ingest Docusaurus content to Qdrant
│   │   ├── personalization_service.py # Look up user background, rewrite prompts
│   │   ├── translation_service.py # Async Urdu translation for chapters
│   │   ├── llm.py                 # OpenAI API wrapper + ChatKit SDK wrapper
│   │   ├── database.py            # Neon PostgreSQL ORM (users, preferences, sessions)
│   │   └── cache.py               # Redis cache for frequent queries + translations
│   ├── models/
│   │   ├── schemas.py             # Pydantic schemas (ChatRequest, PersonalizeRequest, etc.)
│   │   └── db.py                  # SQLAlchemy models (User, UserPreferences, ChatSession, etc.)
│   ├── config/
│   │   └── settings.py            # Environment variables (Better-Auth secret, API keys, etc.)
│   └── main.py
├── tests/
│   ├── unit/
│   │   ├── test_rag_agent.py
│   │   └── test_api.py
│   ├── integration/
│   │   └── test_chatbot_e2e.py
│   └── contract/
│       └── test_openapi.py
├── pyproject.toml                 # Pinned dependencies
├── Dockerfile
├── docker-compose.yml             # FastAPI + Qdrant + PostgreSQL locally
└── .env.example

examples/
├── module-1-ros2/
│   ├── lesson-1-setup/
│   │   ├── package.xml
│   │   ├── launch/setup.launch.py
│   │   ├── src/
│   │   └── README.md
│   ├── lesson-2-launch/
│   └── devcontainer.json
├── module-2-control/
├── module-3-perception/
├── module-4-sim2real/
├── docker/                        # Shared Docker images
│   ├── Dockerfile.ros2-humble
│   └── Dockerfile.gazebo
└── ci/
    └── validate_examples.sh       # GitHub Actions script to test all code examples

.github/
├── workflows/
│   ├── docusaurus-build.yml       # Build and deploy Docusaurus to GitHub Pages
│   ├── backend-test.yml           # Test FastAPI, chatbot latency
│   ├── validate-examples.yml      # Test all ROS 2 code examples in Docker
│   └── cost-tracking.yml          # Log Qdrant/OpenAI costs
├── issue_template/
└── pull_request_template.md
```

**Structure Decision**:
- **Frontend**: Docusaurus 3.x + React (TypeScript) for site, embedded React component for chatbot widget.
- **Backend**: FastAPI monolith (single repo, can separate later); OpenAI Agents SDK for reasoning; Qdrant Python SDK for vector retrieval.
- **Examples**: ROS 2 packages per module in separate `examples/` folder (can be spun into git submodule if >50 MB).
- **Deployment**: Docusaurus via GitHub Pages (CI/CD), FastAPI on cloud VM or container service (AWS EC2, Google Cloud Run, or Anthropic Bedrock inference).

---

## Phased Implementation Roadmap

### Phase 0: Foundation & Research (Weeks 1-2, Complete by Dec 13)

**Objectives**: Resolve unknowns, finalize tech stack decisions, set up CI/CD + repositories.

**Deliverables**:
- ✅ `research.md` (unknowns resolved)
- ✅ GitHub repository structure initialized (docusaurus/, backend/, examples/)
- ✅ GitHub Actions workflows scaffolded (build, test, deploy)
- ✅ Docker images built (ROS 2 Humble + Gazebo, FastAPI skeleton)
- ✅ Neon PostgreSQL and Qdrant Cloud instances provisioned (credentials in `.env.example`)
- ✅ OpenAI Agents SDK evaluated (version pinned in requirements.txt)

**Dependencies**:
- None (foundational phase)

**Research Unknowns to Resolve**:
1. **OpenAI Agents SDK API stability** → Verify latest stable version, test with sample queries, document fallback to Chat Completions API
2. **Qdrant Cloud pricing scaling** → Test with 10k embeddings, estimate monthly cost
3. **GitHub Pages deployment limits** → Test Docusaurus build artifact size, confirm <1 GB
4. **Jetson Orin Nano CUDA compatibility** → Verify ROS 2 Humble works on Jetson + document cuda-12.0 version parity

---

### Phase 1: Infrastructure, Auth & Contracts (Weeks 2-3, Complete by Dec 20)

**Objectives**: Finalize data models, API contracts, authentication system, deployment pipeline, and quickstart guide. **BONUS FEATURE**: Better-Auth integration (50 pts).

**Deliverables**:
- ✅ `data-model.md` (entities, relationships, DB schema including users + preferences)
- ✅ `contracts/chatbot-api.yaml` (OpenAPI spec for `/api/ask` with user_context, `/api/personalize/*`, `/api/translate/*`, `/api/auth/*`)
- ✅ `contracts/embedding-schema.json` (Qdrant collection schema with user metadata filtering)
- ✅ `contracts/auth-schema.yaml` (Better-Auth signup/login payloads: hardware_background, software_background)
- ✅ `quickstart.md` (dev environment setup, local Docker Compose with Better-Auth)
- ✅ Backend skeleton (FastAPI app with `/health`, `/api/ask`, `/api/auth/signup`, `/api/auth/login` stubs)
- ✅ **Better-Auth Setup** (50 pts):
  - Better-Auth library installed and configured in FastAPI
  - `/api/auth/signup` endpoint accepting hardware_background + software_background
  - `/api/auth/login` endpoint with secure session management
  - User preferences table schema in Neon (personalization_enabled, language, theme)
  - Better-Auth secrets securely stored in `.env`
- ✅ Docusaurus skeleton (site structure, sidebar config, AuthModal component, PersonalizeButton placeholder)
- ✅ GitHub Actions workflows operational (build validation, cost tracking, auth tests)
- ✅ Database schema deployed to Neon:
  - `users` table: id, github_id, email, hardware_background, software_background, created_at, updated_at
  - `user_preferences` table: user_id, personalization_enabled, language, theme, updated_at
  - `chat_sessions` table: id, user_id, query_count, context_tokens, created_at, messages JSONB
  - `deployment_logs` table: user_id, action, timestamp, status

**Dependencies**:
- Requires Phase 0 complete (repositories, Docker images, cloud services provisioned)
- Better-Auth library version pinned in requirements.txt

---

### Phase 2: Pilot Content & Core Agents (Weeks 3-4, Complete by Dec 27)

**Objectives**: Author Module 1 content, implement RAG pipeline, personalization engine, and Urdu translation service. Test chatbot end-to-end with personalization and translation. **BONUS FEATURES**: Personalization (50 pts) + Urdu Translation (50 pts).

**Deliverables**:
- ✅ Module 1 (ROS 2 Foundations & Gazebo Simulation) fully written:
  - 10-15 lessons (Markdown files, max 3000 tokens each)
  - YAML frontmatter with metadata (prerequisites, hardware_tier, learning_objectives, estimated_time)
  - Code examples validated in Docker (100% runnable)
  - 1 end-to-end demo video (Gazebo sim running, Jetson deployment)
- ✅ Embedding ingestion pipeline (`.src/services/embedding_ingest.py`):
  - Reads Docusaurus content, chunks into <3000 token documents
  - Generates OpenAI embeddings, uploads to Qdrant
  - Cost tracking + batch processing
- ✅ RAG agent implementation (`src/agents/rag_agent.py`):
  - Integrates OpenAI Agents SDK
  - Query → Vector search (Qdrant) → LLM reasoning → JSON response
  - Handles multi-turn context (session state in PostgreSQL)
- ✅ Chatbot API operational (`/api/ask`, rate limiting, error handling)
- ✅ Chatbot widget integrated into Docusaurus sidebar
- ✅ Latency test (100 queries, p95 ≤3s target)
- ✅ Accuracy test (20 FAQ curated, 90% correct answers without hallucination)

**Dependencies**:
- Phase 1 complete (infrastructure, contracts, quickstart)
- OpenAI Agents SDK stable (Phase 0 research)

---

### Phase 3: Full Content & Hardening (Weeks 4-5, Complete by Nov 23)

**Objectives**: Complete Modules 2-4 + Capstone, performance optimization, pre-launch hardening.

**Deliverables**:
- ✅ Module 2 (Control Systems & Motion Planning):
  - 10-12 lessons (Markdown + code examples)
  - 1 end-to-end demo (MoveIt trajectory planning on simulated arm)
- ✅ Module 3 (Embodied Perception & Vision):
  - 10-12 lessons (Gazebo sensors, NVIDIA Isaac integration)
  - 1 end-to-end demo (Vision-based sim2real transfer)
- ✅ Module 4 (Sim-to-Real Transfer & Deployment):
  - 8-10 lessons (deployment to Jetson Orin, RTX workstations)
  - 1 end-to-end demo (Gazebo → physical deployment)
- ✅ Capstone (End-to-End Humanoid Project):
  - 5-8 lessons (capstone project spec, grading rubric)
- ✅ All code examples tested in Docker + GitHub Actions CI
- ✅ Chatbot indexed with ≥90% of curriculum content
- ✅ Latency optimization (p95 ≤3s verified over 1,000 queries)
- ✅ Uptime test (99.5% SLA measured over 7 days)
- ✅ Load test (≥50 concurrent chatbot requests)
- ✅ Documentation complete (README, CONTRIBUTING, troubleshooting guides)
- ✅ Pre-launch checklist (14 items verified)

**Dependencies**:
- Phase 2 complete (Module 1 pilot, RAG agent, chatbot API)

---

### Phase 4: Launch & Post-Launch (Week 5 onward, by Nov 30)

**Objectives**: Deploy to production, monitor, and prepare for post-launch support.

**Deliverables**:
- ✅ Docusaurus site deployed to GitHub Pages (public URL live)
- ✅ FastAPI backend deployed to cloud (AWS EC2, Google Cloud Run, or Anthropic Bedrock)
- ✅ Qdrant Cloud instance scaled for production
- ✅ Monitoring dashboard (latency, uptime, costs, error rates)
- ✅ Incident response runbook (common issues + fixes)
- ✅ Community channels (GitHub Issues, Discussions) active
- ✅ ≥100 GitHub stars achieved (if possible; otherwise flag for v1.1)

**Dependencies**:
- Phase 3 complete (full curriculum, hardening)

---

## Key Design Decisions

### D1: Technology Stack Justification

| Component | Choice | Rationale | Alternatives Rejected |
|-----------|--------|-----------|----------------------|
| **Frontend (Docs)** | Docusaurus 3.x | MDX support for interactive content; GitHub Pages deployment; active community | MkDocs (limited MDX), Sphinx (Python-centric, overkill for docs) |
| **Frontend (Chat UI)** | React + TypeScript | Native integration with Docusaurus; component reusability | Vue (less native), vanilla JS (harder to maintain) |
| **Backend API** | FastAPI 0.104+ | Async, OpenAPI auto-docs, minimal boilerplate; Pydantic for validation | Django REST (heavier, overkill), Flask (less structured) |
| **LLM Agent** | OpenAI Agents SDK | Latest reasoning capabilities; streaming support; production-ready | Anthropic Claude API (less mature at this date), open-source agents (latency risk) |
| **Vector DB** | Qdrant Cloud | Managed service (no ops), superior filtering, cost-effective at scale | Pinecone (higher cost), Weaviate (self-hosted complexity), pgvector (overkill for this scale) |
| **SQL Store** | Neon PostgreSQL | Managed, serverless, auto-scaling; git-like branching; cheap | RDS (cold start latency), Firebase (vendor lock-in), DynamoDB (harder to query) |
| **CI/CD** | GitHub Actions | Native to repository; free tier sufficient; mature ecosystem | GitLab CI (repo switching), CircleCI (cost), Jenkins (self-hosted ops) |
| **IaC** | Terraform (future) | For Phase 1.1; manual setup acceptable for v1.0 deadline | CloudFormation (AWS-only), Pulumi (over-engineered for v1.0) |

### D2: Content Structure & RAG Approach

**Markdown Chunking**: Max 3000 tokens/file → ensures each lesson fits in a single Qdrant search result + LLM context window. Prevents "answer split across 2 files" confusion.

**Embedding Strategy**:
- Batch ingest: Run nightly (not real-time) to control OpenAI costs.
- Re-embed on content change: Triggered by CI/CD (push to `docs/` directory).
- Caching: Cache frequently queried lesson chunks in Redis (future optimization).

**Multi-Turn Context**:
- Session state stored in PostgreSQL (conversation history).
- OpenAI Agents SDK manages context window management (not manual).
- Max session length: 10 turns (to prevent context explosion).

### D3: Deployment Architecture

**Docusaurus**: GitHub Pages (no backend needed; static hosting).
**FastAPI + Qdrant**: Cloud VM or container service:
  - **Option A (v1.0 default)**: AWS EC2 `t3.medium` with Docker Compose (manual scaling via replicas).
  - **Option B (v1.1 future)**: Google Cloud Run (serverless, auto-scaling).
  - **Option C (future)**: Anthropic Bedrock inference (if available).

**Database**: Neon PostgreSQL serverless (auto-scaling, <1s cold start acceptable for chatbot).

**Monitoring**: CloudWatch (AWS) or Stackdriver (Google Cloud) for latency/uptime/error tracking.

---

## API & Data Model Preview

### REST API: Chatbot (`/api/ask`)

```yaml
POST /api/ask
Description: Submit natural language query, receive RAG-powered answer
Request:
  query: string (5-500 characters)
  session_id: string (optional, for multi-turn context)
  hardware_tier: enum (simulation_only|cpu|gpu|jetson, optional filter)

Response (200 OK):
  answer: string
  sources: array[{lesson_id: string, title: string, url: string}]
  confidence: float (0-1, hallucination risk score)
  session_id: string (for follow-up queries)

Error (400 Bad Request):
  error_code: string (e.g., "invalid_query_length")
  message: string

Error (503 Service Unavailable):
  error_code: "chatbot_overloaded"
  retry_after: integer (seconds)
```

### Database Schema (PostgreSQL)

```sql
-- Users (optional, GitHub OAuth only)
CREATE TABLE users (
  id UUID PRIMARY KEY,
  github_id INTEGER UNIQUE,
  username STRING,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Chatbot sessions (multi-turn context)
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id),
  query_count INTEGER,
  created_at TIMESTAMP,
  last_activity TIMESTAMP,
  messages JSONB  -- Array of {role: "user"|"assistant", content: string}
);

-- Lesson metadata (imported from Docusaurus frontmatter)
CREATE TABLE lessons (
  id STRING PRIMARY KEY,
  title STRING,
  module STRING,
  hardware_tier STRING,
  prerequisites JSONB,
  learning_objectives JSONB,
  estimated_time_minutes INTEGER,
  last_updated TIMESTAMP
);

-- Vector embeddings index (Qdrant, not PostgreSQL)
```

---

## Risks & Mitigations

| Risk | Severity | Mitigation |
|------|----------|-----------|
| OpenAI Agents SDK breaking changes before Nov 30 | Medium | Pin version; test monthly; maintain fallback to Chat Completions API in code |
| Qdrant Cloud cost spike (unexpected query volume) | Medium | Implement rate limiting (100 req/min per IP); cost dashboard; kill switch (pause ingestion if >$600/month) |
| GitHub Pages storage exceeds 1 GB | Low | Compress assets; defer videos to YouTube/Vimeo; test Docusaurus build size in Phase 0 |
| Jetson Orin Nano CUDA/ROS 2 incompatibility | Low | Test early in Phase 1; document fallback to RTX-only lessons; create GitHub issue if blocker |
| Module content not complete by Nov 23 (deadline buffer) | High | Prioritize Modules 1-2 in Phase 2; Modules 3-4 can reduce scope (fewer lessons, pilot demos only) |
| Chatbot accuracy <90% (hallucination) | Medium | Curate FAQ test set in Phase 2; implement confidence scoring; monitor user feedback post-launch |
| Concurrent user load exceeds 50 | Low | Horizontal scaling (2+ FastAPI replicas); Qdrant can auto-scale in cloud tier |

---

## Success Criteria (Measurable)

By November 30, 2025, **ALL** of the following must be met:

- ✅ Docusaurus site live on GitHub Pages (0 build errors, <2s load time)
- ✅ FastAPI `/api/ask` endpoint operational (sample queries working)
- ✅ Qdrant index >90% curriculum coverage (verified via query test)
- ✅ Chatbot latency: p95 ≤3s (1,000 query test logged)
- ✅ Chatbot accuracy: 90% of FAQ test set correct (graded by human + LLM rubric)
- ✅ All code examples runnable in Docker (GitHub Actions CI green)
- ✅ ≥3 end-to-end sim-to-real demo videos (recorded, embedded in docs)
- ✅ Uptime: 99.5% over 7-day test window (no unscheduled downtime)
- ✅ No "TODO," "FIXME," or placeholders in released content
- ✅ README + CONTRIBUTING.md complete and reviewed
- ✅ ≥100 GitHub stars (if possible; tracked weekly)

---

## References & Next Steps

**Related Artifacts**:
- Constitution: `.specify/memory/constitution.md` (v1.0.0)
- Specification: `specs/textbook-v1/spec.md` (complete)
- Research Phase 0 Output: `specs/textbook-v1/research.md` (pending)
- Data Model Phase 1 Output: `specs/textbook-v1/data-model.md` (pending)
- API Contracts Phase 1 Output: `specs/textbook-v1/contracts/` (pending)
- Task Breakdown Phase 2 Output: `specs/textbook-v1/tasks.md` (pending via `/sp.tasks`)

**Next Command**:
Run `/sp.tasks` to generate `specs/textbook-v1/tasks.md` with dependency-ordered task breakdown and sprint allocation.

**ADR Opportunities**:
- 📋 Technology Stack Decision (Docusaurus + FastAPI + Qdrant)
- 📋 Content Chunking Strategy (3000-token max per file)
- 📋 Multi-Turn Conversation State Management (PostgreSQL sessions)

---

## Revision History

| Version | Date | Author | Change |
|---------|------|--------|--------|
| 1.0 | 2025-12-06 | AI Architect | Initial plan; 4-phase roadmap, architecture overview, risk mitigation |
/
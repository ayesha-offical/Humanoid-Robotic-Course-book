# Tasks: Physical AI & Humanoid Robotics Textbook Platform (v1.0)

**Input**: Design documents from `/specs/textbook-v1/`
**Prerequisites**: plan.md, spec.md
**Total Tasks**: 104 atomic tasks (15-30 mins each)
**Target**: Core RAG Chatbot Implementation
**Note**: Phase 2.5 (FastAPI Implementation) is NEW and BLOCKING for Phase 3

**Organization**: Tasks grouped by phase to enable independent, parallel implementation.

---

## Format: `[ ] [TaskID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Exact file paths included in descriptions

---

## Phase 1: Setup & Project Initialization

**Purpose**: Project structure, repositories, CI/CD scaffolding

**Timeline**: Weeks 1-2, Complete by Dec 13

- [ ] T001 Create project structure per plan (docosaurus/, backend/, examples/) in repository root
- [ ] T002 [P] Initialize Docosaurus 3.x project with MDX support in `docosaurus/`
- [ ] T003 [P] Initialize FastAPI 0.104+ backend project in `backend/` with pyproject.toml
- [ ] T004 [P] Initialize ROS 2 examples repository structure in `examples/` with package.xml templates
- [ ] T005 [P] Create .github/workflows/ directory structure for CI/CD pipelines
- [ ] T006 [P] Create .env.example template with all required API keys (OpenAI, Qdrant)
- [ ] T007 Create .gitignore to exclude .env, __pycache__, node_modules, .DS_Store
- [ ] T008 [P] Configure linting tools: ESLint for TypeScript, black/flake8 for Python
- [ ] T009 [P] Configure pre-commit hooks in .pre-commit-config.yaml (format, lint, secrets scan)
- [ ] T010 Create README.md with quick-start instructions (clone → docker build → docker run)
- [ ] T011 Create CONTRIBUTING.md with commit message style guide + PR template
- [ ] T012 Initialize git repository + create branch `1-textbook-platform` (if not exists)

**Checkpoint**: Project structure ready; CI/CD pipeline scaffolded.

---

## Phase 2: Foundational Infrastructure (Blocking Prerequisites)

**Purpose**: Core infrastructure REQUIRED before ANY work can start

**⚠️ CRITICAL**: Core API work cannot begin until Phase 2 is complete

### P2.1: Backend API Infrastructure

- [ ] T013 [P] Setup FastAPI main application in `backend/src/main.py` with CORS, middleware
- [ ] T014 [P] Create FastAPI router structure: `backend/src/api/routers/` (chat, health, admin)
- [ ] T015 [P] Implement logging framework in `backend/src/config/logging.py` with request/response logging
- [ ] T016 [P] Implement error handling middleware in `backend/src/middleware/error_handler.py` (HTTPException, validation, 500 errors)
- [ ] T017 [P] Setup environment configuration loader in `backend/src/config/settings.py` (Pydantic BaseSettings)
- [ ] T018 [P] Create request/response Pydantic schemas in `backend/src/models/schemas.py` (ChatRequest, ChatResponse)

### P2.2: Vector Database & LLM Integration

- [ ] T019 [P] Provision Qdrant Cloud instance + collection schema (curriculum_embeddings) in .env
- [ ] T020 [P] Provision OpenAI API key (Agents SDK + ChatKit SDK usage) in .env
- [ ] T021 [P] Create Qdrant client wrapper in `backend/src/services/qdrant.py` (connect, query, upsert)
- [ ] T022 [P] Create OpenAI LLM wrapper in `backend/src/services/llm.py` (embeddings, chat completions, agents)

### P2.3: Frontend Infrastructure

- [ ] T023 [P] Setup Docosaurus configuration in `docosaurus/docusaurus.config.js` (title, URL, theme, sidebar)
- [ ] T024 [P] Create React custom hooks directory structure in `docosaurus/src/hooks/` (useChat.ts)
- [ ] T025 [P] Create Docosaurus theme layout overrides in `docusaurus/src/theme/` for header + sidebar
- [ ] T026 [P] Setup TailwindCSS in `docosaurus/` for styling (tailwind.config.js)
- [ ] T027 [P] Create `.env.local` template for frontend (API_URL, etc.) in `docosaurus/`

### P2.4: GitHub Actions CI/CD

- [ ] T028 Create `.github/workflows/backend-test.yml`: Run pytest + type checking on backend/ push
- [ ] T029 Create `.github/workflows/frontend-build.yml`: Build Docosaurus + Lighthouse score on docosaurus/ push
- [ ] T030 Create `.github/workflows/deploy-pages.yml`: Deploy Docosaurus to GitHub Pages on main branch merge
- [ ] T031 Create `.github/workflows/docker-build.yml`: Build + push backend Docker image to Docker Hub/GHCR
- [ ] T032 [P] Create `.github/workflows/validate-code-examples.yml`: Test all ROS 2 code examples in Docker

### P2.5: Docker & Deployment

- [ ] T033 Create `backend/Dockerfile` with Python 3.10 + FastAPI + dependencies (multi-stage build)
- [ ] T034 Create `backend/docker-compose.yml` with FastAPI, Qdrant, Redis services
- [ ] T035 Create `docosaurus/Dockerfile` for Node.js + Docosaurus build
- [ ] T036 Create `examples/docker/Dockerfile.ros2-humble` with ROS 2 Humble + Gazebo base image
- [ ] T037 Create example `.devcontainer.json` for VSCode development in both backend/ and docosaurus/

**Checkpoint**: Foundation ready - core RAG work can start ✅

---

## Phase 2.5: FastAPI Implementation (NEW - BLOCKING)

**Purpose**: Expose existing RAG agent logic via FastAPI backend

**Timeline**: Immediate, blocking for Phase 3

**Dependencies**: Existing `backend/rag_agent.py` and embedding logic must be complete

### P2.5.1: Pydantic Models & Request/Response Schemas

- [X] T097 Create `backend/models.py` with Pydantic models for Chat Request/Response:
  - `ChatRequest`: fields: query (str), session_id (optional str)
  - `ChatResponse`: fields: answer (str), sources (list), confidence (float), latency_ms (int)
  - `HealthResponse`: fields: status (str), uptime_seconds (int)
  - Add input validation (query length limits, type checking)

### P2.5.2: Router & Endpoints

- [X] T098 Create `backend/router.py` with FastAPI APIRouter:
  - `POST /chat`: Accept ChatRequest, call RAG agent, return ChatResponse
  - Input validation: query length (5-2000 chars), error handling
  - Integrate existing `rag_agent.py` logic into endpoint handler
  - Add request/response logging

### P2.5.3: FastAPI Application Entry Point

- [X] T099 Create `backend/api.py` (main FastAPI app):
  - Initialize FastAPI app with title, description, version
  - Configure CORS middleware to allow Docosaurus frontend access:
    - Allow origins: `http://localhost:3000`, `http://localhost:8000`, production domain
    - Allow methods: GET, POST, OPTIONS
    - Allow headers: Content-Type, Authorization
  - Mount router (T098) at `/api`
  - Add health check endpoint `GET /health`
  - Add error handling middleware for global exception catching
  - Configure logging and request tracking

### P2.5.4: RAG Agent Integration

- [X] T100 [P] Integrate `backend/rag_agent.py` into `backend/router.py`:
  - Import existing Agent class from rag_agent.py
  - Create wrapper function in router: `query_rag_agent(query: str) -> dict`
  - Handle async execution if agent is async
  - Extract answer, sources, confidence from agent response
  - Add error handling for agent failures (timeout, API errors)
  - Add request timeout protection (max 30 seconds per query)

- [X] T101 [P] Create `backend/config.py`:
  - Load environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
  - Configure logger settings
  - Set request timeouts, CORS origins, API rate limits
  - Use Pydantic Settings for type-safe configuration

### P2.5.5: Testing & Validation

- [X] T102 Create unit tests in `backend/tests/test_api.py`:
  - Test POST /chat with valid query → verify ChatResponse structure
  - Test POST /chat with invalid query (too short/long) → verify validation errors
  - Test POST /chat endpoint handles rag_agent errors gracefully
  - Test GET /health endpoint returns healthy status

- [X] T103 Create integration test `backend/tests/test_integration.py`:
  - Start FastAPI app (uvicorn)
  - Send 5 sample queries to POST /chat
  - Verify responses have all required fields
  - Verify CORS headers are set correctly
  - Verify latency is <5 seconds per query

**Checkpoint**: FastAPI backend exposes RAG agent, ready for frontend integration ✅

---

## Phase 3: Core RAG Chatbot Implementation

**Goal**: Implement RAG pipeline and chatbot API

**Independent Test**: User can query `/api/ask` and receive answers from curriculum content

### P3.1: Content Ingestion Pipeline

- [ ] T038 [P] Create content chunking service in `backend/src/services/chunking.py` (split markdown by topic, <3000 tokens)
- [ ] T039 [P] Create embedding generation service in `backend/src/services/embeddings.py` (OpenAI API wrapper)
- [ ] T040 [P] Create batch ingestion script in `backend/scripts/ingest_curriculum.py` (read markdown → chunk → embed → store in Qdrant)
- [ ] T041 [P] Implement metadata extraction in `backend/src/services/metadata.py` (extract title, module, topic from YAML frontmatter)
- [ ] T042 Create unit tests for chunking service in `backend/tests/test_chunking.py`
- [ ] T043 Create unit tests for embeddings service in `backend/tests/test_embeddings.py`

### P3.2: RAG Agent & Retrieval

- [ ] T044 [P] Create Qdrant retriever in `backend/src/agents/retrievers.py` (similarity search with metadata filtering)
- [ ] T045 [P] Create OpenAI Agents SDK integration in `backend/src/agents/rag_agent.py` (define tools, system prompt, function calling)
- [ ] T046 [P] Implement response formatting in `backend/src/agents/formatters.py` (structured JSON output with sources)
- [ ] T047 Create unit tests for retriever in `backend/tests/test_retriever.py`
- [ ] T048 Create unit tests for RAG agent in `backend/tests/test_rag_agent.py`

### P3.3: Chatbot API

- [ ] T049 [P] Implement `/api/ask` endpoint in `backend/src/api/routers/chat.py` (accept query → retrieve → agent → respond)
- [ ] T050 [P] Implement `/api/ask/stream` endpoint for streaming responses via ChatKit SDK
- [ ] T051 [P] Implement request validation in chat router (query length, rate limiting)
- [ ] T052 [P] Implement response caching in `backend/src/services/cache.py` (Redis)
- [ ] T053 Create integration tests for chat API in `backend/tests/test_chat_api.py` (10+ queries)
- [ ] T054 Create latency benchmark script in `backend/scripts/benchmark_latency.py` (100 queries, measure p95)

### P3.4: Frontend Chatbot Widget

- [ ] T055 [P] Create ChatbotWidget component in `docosaurus/src/components/ChatbotWidget.tsx` (message list, input, send button)
- [ ] T056 [P] Create useChat hook in `docosaurus/src/hooks/useChat.ts` (call `/api/ask`, manage state)
- [ ] T057 [P] Implement streaming support in useChat hook (WebSocket or SSE)
- [ ] T058 [P] Integrate ChatbotWidget into Docosaurus Root theme in `docosaurus/src/theme/Root.tsx`
- [ ] T059 Create snapshot tests for ChatbotWidget in `docosaurus/tests/ChatbotWidget.test.tsx`

**Checkpoint**: Basic RAG chatbot working end-to-end ✅

---

## Phase 4: Pilot Content & Optimization

**Goal**: Module 1 content + performance tuning

### P4.1: Module 1 Content Authoring

- [ ] T060 [P] Write Module 1 Lesson 1: ROS 2 Installation (markdown + code example)
- [ ] T061 [P] Write Module 1 Lesson 2: ROS 2 Concepts (publisher/subscriber, services)
- [ ] T062 [P] Write Module 1 Lesson 3: Gazebo Basics (launch file, simulation)
- [ ] T063 [P] Write 10-15 additional Module 1 lessons (detailed in content spec)
- [ ] T064 Create code example test suite for Module 1 in `examples/module1/tests/`
- [ ] T065 Record 1 demo video: Gazebo simulation of mobile robot (30-60 min content)

### P4.2: Performance Tuning

- [ ] T066 Profile chatbot latency: Identify bottlenecks (retrieval vs LLM inference)
- [ ] T067 Optimize Qdrant queries: Add indexing, adjust batch sizes
- [ ] T068 Optimize embeddings caching: Pre-compute for popular queries
- [ ] T069 Implement request queuing in FastAPI (handle >50 concurrent requests)
- [ ] T070 Run latency test: 100 queries, target p95 ≤3s

**Checkpoint**: Module 1 ready, chatbot performance verified ✅

---

## Phase 5: Full Content & Hardening

**Goal**: Modules 2-4 + Capstone + production readiness

### P5.1: Additional Content

- [ ] T071 Write Module 2: Control Systems (10-15 lessons)
- [ ] T072 Write Module 3: Embodied Perception (10-15 lessons)
- [ ] T073 Write Module 4: Sim-to-Real Transfer (10-15 lessons)
- [ ] T074 Write Capstone: End-to-End Humanoid Project (5-10 lessons)
- [ ] T075 Record 2 additional demo videos (Modules 2 & 3)

### P5.2: Quality Assurance

- [ ] T076 Run full test suite: pytest + coverage >80%
- [ ] T077 Test all code examples in Docker (Module 1-4)
- [ ] T078 Verify all markdown files build with Docosaurus (0 warnings)
- [ ] T079 Run 1,000-query latency test, target p95 ≤3s
- [ ] T080 Run 7-day uptime test, target 99.5% availability
- [ ] T081 Run load test: ≥50 concurrent queries
- [ ] T082 Verify Qdrant index coverage: >90% of curriculum

### P5.3: Documentation

- [ ] T083 Finalize README.md with complete setup instructions
- [ ] T084 Finalize CONTRIBUTING.md with development workflow
- [ ] T085 Create TROUBLESHOOTING.md with common issues + fixes
- [ ] T086 Create API documentation (OpenAPI spec + examples)
- [ ] T087 Create DEPLOYMENT.md (AWS, Google Cloud, manual setup)

**Checkpoint**: All content ready, all tests passing ✅

---

## Phase 6: Launch & Post-Launch

**Goal**: Production deployment + monitoring

### P6.1: Deployment

- [ ] T088 Deploy FastAPI backend to AWS EC2 or Google Cloud Run
- [ ] T089 Scale Qdrant Cloud for production
- [ ] T090 Deploy Docosaurus site to GitHub Pages
- [ ] T091 Setup monitoring dashboard (latency, uptime, costs)
- [ ] T092 Setup alerting (Slack/email for errors, SLA breaches)

### P6.2: Community & Feedback

- [ ] T093 Publish GitHub repository + create community channels (Issues, Discussions)
- [ ] T094 Share project on social media / AI/robotics communities
- [ ] T095 Collect user feedback (chatbot accuracy, content clarity)
- [ ] T096 Track GitHub stars + engagement metrics

**Checkpoint**: Live on production, monitoring active ✅

---

## Revision History

| Version | Date | Author | Change |
|---------|------|--------|--------|
| 1.0 | 2025-12-06 | AI Architect | Initial tasks; core RAG implementation without auth/database |

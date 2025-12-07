# Physical AI & Humanoid Robotics Textbook Constitution

## Global Vision

Bridge the gap between digital AI brains and physical embodied systems by creating an AI-native,
hands-on textbook that teaches Sim-to-Real principles. Students and developers transition from
software-only AI to Embodied Intelligence through practical code examples, reproducible environments
(ROS 2, Gazebo, NVIDIA Isaac), and RAG-friendly documentation suitable for AI agent ingestion.

Target audience: Educators, undergraduate/graduate students, AI researchers, and roboticists moving
into embodied intelligence.

---

## Core Principles

### I. Hands-On First

Every lesson MUST include working code examples, not theory alone. Code examples must be
reproducible with clear prerequisites (hardware requirements, software dependencies, estimated runtime).
Students should be able to copy-paste and run within 15 minutes of setup. Simulation-first
approach: Gazebo before hardware; code works in sim before deployment to real robots.
- Rationale: Embodied AI is learned by doing; theory without practice defeats the purpose.

### II. Reproducible Code & Environments

All code examples MUST use versioned dependencies (pyproject.toml, package.xml with pinned versions).
Every tutorial chapter includes a Docker image or devcontainer.json for identical environment across
machines. ROS 2 Humble baseline (Ubuntu 22.04 + CUDA 12.0 for GPU examples).
Gazebo simulations must be deterministic (fixed random seeds for reproducibility).
- Rationale: Robotics requires exact reproduction; environment drift breaks learning and deployment.

### III. Sim-to-Real Pipeline

Every robotics module must demonstrate the pipeline: Gazebo simulation → transfer to real hardware
(Jetson Orin or RTX workstation). Document sim2real gaps (friction, latency, actuator limits).
Include failure cases: "This works in sim but fails on hardware because..." and remediation steps.
- Rationale: Sim-to-real transfer is the core challenge in embodied AI; students must learn it early.

### IV. RAG-Friendly Structure

All documentation MUST be structured for AI agent ingestion (OpenAI Agents SDK, Retrieval-Augmented
Generation). Markdown files split by topic (max 3000 tokens per file); include YAML frontmatter with
metadata (prerequisites, learning objectives, hardware tier). Code blocks tagged with language and
purpose (e.g., ```python:setup, ```bash:deployment).
- Rationale: Future students will use AI chatbots to navigate the textbook; searchability & structure matter.

### V. Quality Code Standards

All Python code follows PEP 8; docstrings mandatory for all functions (Google style).
Type hints required for public APIs. ROS 2 nodes must include launch files and integration tests.
No hardcoded paths; use environment variables and configuration files.
Security: Never commit secrets; use .env files with examples (.env.example).
- Rationale: Production-ready code teaches professional practice; students deploy these to real robots.

### VI. Practical Prerequisites & Hardware Tiers

Every lesson explicitly lists hardware required (Simulation Only / CPU / GPU RTX / Jetson Orin).
Software prerequisites: ROS 2 version, Python version, required packages. Estimated time to complete.
Optional: Hardware-specific tips for different platforms.
- Rationale: Students have varying hardware; clarity prevents frustration and failed attempts.

### VII. Assessment & Checkpoints

Each module includes acceptance criteria (runnable tests, simulation benchmarks, deployment checklists).
No vague learning outcomes; specify measurable goals: "Run trajectory planning in <100ms on Jetson" or
"Deploy policy to real arm without manual tuning."
- Rationale: Embodied AI is not just theory; progress is measurable through actual performance.

---

## Technology & Format Standards

### Frontend: Docusaurus + Markdown

All lesson content authored in Markdown with MDX extensions for interactive simulations.
Structure: `/docs/<topic>/<lesson>.md` (one lesson = one file, max 3000 tokens).
Docusaurus sidebar config in `/docusaurus.config.js`; auto-generate from file structure.
Deployment: GitHub Pages (CI/CD via GitHub Actions on main branch).
Versioning: Docs versioned alongside software releases (e.g., v1.0, v1.1).

### Backend: OpenAI Agents SDK + FastAPI + Neon Postgres + Qdrant Cloud

FastAPI serves lesson embeddings and student progress tracking (vector DB: Qdrant Cloud).
Neon (PostgreSQL) stores user submissions, quiz results, and deployment logs.
OpenAI Agents SDK powered chatbot ingests Docusaurus content for live Q&A support.
All API endpoints return JSON; include comprehensive error messages (status codes, error codes, retry hints).

### Code Examples & Repositories

ROS 2 packages structured per lesson in `/examples/<topic>/<lesson>` with:
- `package.xml` (dependency manifest)
- `launch/` folder (ROS 2 launch files)
- `src/` (Python or C++ nodes)
- `tests/` (pytest or gtest)
- `README.md` (setup & run instructions)

Python scripts use `pyproject.toml` (modern packaging) or `requirements.txt` (pinned versions).
Git submodules optional for large simulator packages (Gazebo models, Isaac Sim assets).

### Markdown Frontmatter (YAML)

All lesson files include:
```yaml
---
title: "Lesson Title"
topic: "robotics/control"
difficulty: "beginner|intermediate|advanced"
hardware_tier: "simulation_only|cpu|gpu|jetson"
prerequisites: ["ROS 2 basics", "Python 3.10+"]
learning_objectives:
  - "Understand trajectory planning"
  - "Deploy to real hardware"
estimated_time_minutes: 45
last_updated: "2025-12-06"
---
```

---

## Constraints

### Hardware Requirements

- **Minimum (Simulation-only):** CPU (Intel i7 or equivalent), 16GB RAM, Ubuntu 22.04.
- **GPU Track (Gazebo + Physics):** NVIDIA RTX 4060 or better, 32GB RAM, CUDA 12.0.
- **Production (Real Robots):** Jetson Orin 64GB or RTX workstation + compatible manipulator/mobile base.

### Deployment & Delivery

- Docusaurus site deployed to GitHub Pages (HTTPS only).
- CI/CD: GitHub Actions validates all code examples on every commit (Linux only initially).
- Docker images pushed to Docker Hub or GitHub Container Registry (optional but recommended).
- Backend APIs (FastAPI + Qdrant) deployed to cloud (AWS, Google Cloud, or Anthropic Bedrock for inference).

### Deadlines & Milestones

- **Beta Release (v0.5):** Core 5 modules + simulator + chatbot integration by end of Q1 2026.
- **v1.0 Stable:** All modules, real hardware demos, API stability by Q2 2026.
- **Maintenance:** Quarterly updates to align with ROS 2 Humble LTS; security patches within 2 weeks.

---

## Success Criteria

### Textbook Quality

- ✅ **100% Runnable Code:** Every code snippet must execute without modification (after setup).
- ✅ **Sim-to-Real Verified:** At least 3 modules demonstrate successful transfer to real hardware (Jetson or manipulator).
- ✅ **Student Feedback:** >80% of beta testers successfully complete module 1 within 2 hours of setup.
- ✅ **RAG Readiness:** Chatbot can answer 90% of FAQ without hallucination (measured via user satisfaction).

### Engagement & Impact

- ✅ **Active Community:** >500 GitHub stars, 50+ PRs from external contributors.
- ✅ **Adoption:** Used in ≥3 university robotics programs; cited in ≥5 academic papers.

### Technical Performance

- ✅ **API Latency:** LLM-powered chatbot responds <3 seconds (p95).
- ✅ **Uptime:** 99.5% SLA for deployed APIs (Docusaurus always accessible).
- ✅ **Reproducibility:** All examples run identically on 3 different machines (Docker or devcontainer guarantee).

---

## Governance

This constitution is the authoritative source for all project decisions. All specifications (specs),
implementation plans (plans), and task definitions must align with these principles.

### Amendment Process

1. Identify gap or principle conflict (via GitHub issue labeled `constitution`).
2. Draft amendment with rationale (must reference impacted principles).
3. Circulate to core team (architects, lead educators) for consensus.
4. Bump constitution version (MAJOR.MINOR.PATCH per semver rules):
   - MAJOR: Remove or redefine a core principle.
   - MINOR: Add new principle or expand guidance.
   - PATCH: Clarify wording or fix errors.
5. Update all dependent templates (spec-template.md, plan-template.md, tasks-template.md).
6. Commit with message: `docs(constitution): amend to vX.Y.Z (reason)`

### Compliance & Review

- All PRs must verify alignment with principles (checklist in PR template).
- Code review focuses on: Hands-on quality, reproducibility, RAG structure, security.
- Architecture decisions touching principles → create ADR (Architecture Decision Record).

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06

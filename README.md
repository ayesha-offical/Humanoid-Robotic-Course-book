# Physical AI & Humanoid Robotics Textbook Platform

A unified AI-native platform combining a comprehensive 13-week curriculum with an intelligent RAG chatbot, enabling students and AI agents to transition from software-only AI to hands-on embodied intelligence.

## 🎯 Project Overview

This platform delivers:

1. **Interactive Docosaurus Textbook**: Comprehensive 13-week curriculum covering:
   - Module 1: ROS 2 Foundations & Gazebo Simulation
   - Module 2: Control Systems & Motion Planning
   - Module 3: Embodied Perception & Vision (NVIDIA Isaac, VLA integration)
   - Module 4: Sim-to-Real Transfer & Deployment
   - Capstone: End-to-End Humanoid Project

2. **RAG Chatbot**: FastAPI backend powered by OpenAI Agents SDK with:
   - Qdrant vector database for curriculum content indexing
   - Personalized learning paths based on user expertise
   - Urdu translation support for broader accessibility
   - Real-time Q&A via WebSocket streaming

3. **Production Infrastructure**:
   - GitHub Actions CI/CD
   - Comprehensive testing & monitoring

## 🚀 Quick Start

### Prerequisites

- **System**: Ubuntu 22.04 LTS (or equivalent Linux)
- **Node.js**: 18+ (for Docosaurus frontend)
- **Python**: 3.10+ (for FastAPI backend)
- **Git**: Latest version

### Setup

#### Backend Setup

```bash
cd backend

# Install Python dependencies
pip install -r requirements.txt
# OR using Poetry
poetry install

# Setup environment
cp ../.env.example ../.env
# Edit .env with your API keys

# Run database migrations
alembic upgrade head

# Start FastAPI server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

#### Frontend Setup

```bash
cd docusaurus

# Install Node.js dependencies
npm install
# OR
yarn install

# Copy environment template
cp ../.env.example .env.local

# Edit .env.local with API URL
# REACT_APP_API_BASE_URL=http://localhost:8000/api

# Start Docosaurus dev server
npm start
# Server runs at http://localhost:3000
```

## 📋 First Steps After Setup

### 1. Create Your First User Account

Visit http://localhost:3000 and sign up with:
- Email: `test@example.com`
- Password: Your secure password
- Hardware Background: Select your experience level
- Software Background: Select your coding experience

### 2. Ask the Chatbot a Question

Try these sample questions:
- "How do I install ROS 2 Humble?"
- "What is the difference between MoveIt and Gazebo?"
- "Walk me through the sim-to-real pipeline for a robotic arm"

### 3. Enable Personalization

- Click the "Personalize" toggle in the chatbot widget
- The chatbot will adapt explanations to your experience level

### 4. Try Urdu Translation

- Select "Urdu" from the language switcher in the navbar
- Browse lessons with Urdu translations of key terms

## 🏗️ Project Structure

```
├── backend/                    # FastAPI backend
│   ├── src/
│   │   ├── api/               # API endpoints
│   │   ├── agents/            # RAG & LLM agents
│   │   ├── services/          # Business logic
│   │   ├── models/            # Database & schemas
│   │   ├── config/            # Configuration
│   │   ├── middleware/        # Middleware
│   │   └── utils/             # Utilities
│   ├── tests/                 # Test suites
│   ├── alembic/               # Database migrations
│   └── pyproject.toml         # Python dependencies
│
├── docusaurus/                # Docosaurus frontend
│   ├── src/
│   │   ├── components/        # React components
│   │   ├── hooks/             # Custom React hooks
│   │   ├── theme/             # Docosaurus theme overrides
│   │   └── pages/             # Custom pages
│   ├── docs/                  # Curriculum content (Markdown)
│   ├── docusaurus.config.js   # Docosaurus configuration
│   └── package.json           # Node.js dependencies
│
├── examples/                  # Code examples & ROS 2 packages
│   └── ros2-humble/           # ROS 2 Humble examples
│
├── .github/
│   └── workflows/             # GitHub Actions CI/CD
│
├── .specify/                  # SpecKit Plus templates
├── specs/                     # Design artifacts
├── history/                   # Prompt history & ADRs
│
├── .env.example               # Environment template
├── .gitignore                 # Git ignore rules
├── .pre-commit-config.yaml    # Pre-commit hooks
├── .eslintrc.json             # ESLint configuration
├── .prettierrc.json           # Prettier configuration
└── README.md                  # This file
```

## 🔑 Environment Variables

Create a `.env` file based on `.env.example`:

```bash
# Required
OPENAI_API_KEY=sk-...
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
GOOGLE_TRANSLATE_API_KEY=...

# Optional (defaults provided)
ENVIRONMENT=development
DEBUG=true
LOG_LEVEL=INFO
CHATBOT_MAX_CONTEXT_TOKENS=4000
```

See `.env.example` for complete list of all environment variables.

## 🧪 Testing

### Run All Tests

```bash
cd backend

# Using pytest
pytest tests/

# Using Poetry
poetry run pytest tests/ -v

# With coverage
pytest tests/ --cov=src --cov-report=html
```

### Run Specific Test Suite

```bash
# Unit tests
pytest tests/unit/

# Integration tests
pytest tests/integration/

# Performance benchmarks
pytest tests/performance/ -v

# Contract/API tests
pytest tests/contract/
```

## 📚 API Documentation

Interactive API documentation is available at:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

### Key Endpoints

```bash
# Authentication
POST   /api/auth/signup          # Register new user
POST   /api/auth/login           # User login
POST   /api/auth/logout          # User logout
GET    /api/auth/me              # Get current user profile

# Chatbot
POST   /api/ask                  # Ask chatbot a question
WS     /ws/chat                  # WebSocket for streaming responses

# Personalization
POST   /api/personalize/explain  # Get personalized answer
GET    /api/personalize/profile  # Get user preferences
PUT    /api/personalize/profile  # Update user preferences

# Translation
POST   /api/translate/chapter    # Translate curriculum content
GET    /api/translate/supported-languages

# Admin
GET    /api/admin/metrics        # Platform metrics
GET    /api/admin/users          # User analytics

# Health
GET    /api/health               # System health check
```

## 🔄 Development Workflow

### 1. Create Feature Branch

```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

### 2. Make Changes

```bash
# Edit code, run tests locally
pytest tests/

# Run linting
black backend/
flake8 backend/
isort backend/

# Or use pre-commit hooks (auto-runs before commit)
pre-commit run --all-files
```

### 3. Create Pull Request

```bash
git add .
git commit -m "feat: description of your changes"
# Follow conventional commits: feat:, fix:, docs:, test:, refactor:, etc.

git push origin feature/your-feature-name
# Open PR on GitHub
```

### 4. Merge to Main

After PR review and tests pass, merge to `main` branch. This triggers:
- GitHub Actions CI/CD pipelines
- Automatic deployment to staging
- Optional: Auto-deploy to production

## 🐳 Docker Deployment

### Build Docker Images

```bash
# Backend
docker build -f backend/Dockerfile -t textbook-backend:latest backend/

# Frontend
docker build -f docosaurus/Dockerfile -t textbook-frontend:latest docusaurus/

# Examples (ROS 2)
docker build -f examples/docker/Dockerfile.ros2-humble -t textbook-ros2:latest examples/
```

### Run with Docker Compose

```bash
docker-compose up --build

# View logs
docker-compose logs -f backend
docker-compose logs -f docosaurus

# Stop services
docker-compose down
```

## 📊 Performance Benchmarks

Target metrics:
- **Chatbot Response**: p95 ≤ 3 seconds
- **Personalization Latency**: <500ms overhead
- **Translation Latency**: <1 second per chapter
- **API Uptime**: 99.5% SLA
- **Docosaurus Site Load**: <2 seconds (Lighthouse ≥90)

Run benchmarks:

```bash
cd backend

# Chatbot latency test
pytest tests/performance/test_chatbot_latency.py -v

# Personalization latency test
pytest tests/performance/test_personalization_latency.py -v

# Translation latency test
pytest tests/performance/test_translation_latency.py -v
```

## 🔐 Security

- **Secrets Management**: Use `.env` file (never commit)
- **Database**: Encrypted connections with Neon PostgreSQL
- **Authentication**: JWT tokens via Better-Auth
- **Password Hashing**: bcrypt with 12+ salt rounds
- **API Rate Limiting**: 100 requests/min per user
- **CORS**: Restricted to allowed origins
- **HTTPS**: Enforced in production

Run security audit:

```bash
cd backend

# Bandit security scan
bandit -r src/

# Dependency vulnerability scan
pip audit

# Detect secrets
detect-secrets scan
```

## 📝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed contribution guidelines.

### Code Standards

- **Python**: PEP 8, Google-style docstrings, type hints
- **TypeScript**: ESLint + Prettier
- **Git Commits**: Conventional commits (feat:, fix:, docs:, etc.)
- **Tests**: All PRs must include tests
- **Documentation**: Update docs for new features

## 🐛 Troubleshooting

### Backend Won't Start

```bash
# Check logs
docker-compose logs backend

# Common issues:
# - Missing .env file: cp .env.example .env
# - Database connection: verify DATABASE_URL
# - Port 8000 in use: change in docker-compose.yml

# Reset database
alembic downgrade base
alembic upgrade head
```

### Frontend Won't Load

```bash
# Check Node.js version
node --version  # Should be 18+

# Clear cache and reinstall
rm -rf node_modules package-lock.json
npm install

# Check .env.local configuration
cat .env.local  # Verify REACT_APP_API_BASE_URL
```

### Chatbot Not Responding

```bash
# Verify API is running
curl http://localhost:8000/health

# Check OpenAI API key
echo $OPENAI_API_KEY  # Should not be empty

# Check Qdrant connection
curl -H "API-Key: $QDRANT_API_KEY" $QDRANT_URL/health

# View backend logs
docker-compose logs backend -f
```

## 📞 Support

- **Issues**: Report bugs on [GitHub Issues](https://github.com/ayesha-offical/Humanoid-Robotic-Course-book/issues)
- **Discussions**: Ask questions in [GitHub Discussions](https://github.com/ayesha-offical/Humanoid-Robotic-Course-book/discussions)
- **Email**: support@robotics-textbook.org

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- **ROS 2 Community** for foundational robotics frameworks
- **NVIDIA** for Isaac Sim and robotics resources
- **OpenAI** for GPT models and Agents SDK
- **Qdrant** for vector database infrastructure
- **Neon** for managed PostgreSQL

## 🗺️ Roadmap

### Phase 1 (Dec 6 - Dec 13): Foundation ✅
- [x] Project setup & CI/CD
- [x] FastAPI + Docosaurus skeleton
- [x] Database provisioning

### Phase 2 (Dec 13 - Dec 20): Infrastructure
- [ ] Better-Auth integration (50 pts)
- [ ] Personalization engine (50 pts)
- [ ] Urdu translation service (50 pts)

### Phase 3 (Dec 20 - Dec 27): Core Features
- [ ] RAG chatbot implementation
- [ ] Module 1 content & demos

### Phase 4 (Dec 27 - Nov 30): Completion & Launch
- [ ] Full curriculum content
- [ ] Pre-launch testing
- [ ] Production deployment

**Hard Deadline**: November 30, 2025

## 📊 Metrics

Track platform metrics in real-time:
- **Analytics Dashboard**: http://localhost:8000/admin/dashboard
- **Cost Tracking**: http://localhost:8000/admin/costs
- **Latency Monitoring**: http://localhost:8000/admin/metrics

---

**Last Updated**: December 16, 2025 | **Status**: Phase 1 - Foundation ✅

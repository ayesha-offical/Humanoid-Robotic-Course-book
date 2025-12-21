# Physical AI & Humanoid Robotics Textbook Platform

A unified AI-native platform combining a comprehensive 13-week curriculum with an intelligent RAG chatbot, enabling students and AI agents to transition from software-only AI to hands-on embodied intelligence.

## 🎯 Project Overview

This platform delivers:

1. **🎨 Interactive Docosaurus Textbook**: Comprehensive 13-week curriculum with modern UI/UX:
   - Module 1: ROS 2 Foundations & Gazebo Simulation
   - Module 2: Control Systems & Motion Planning
   - Module 3: Embodied Perception & Vision (NVIDIA Isaac, VLA integration)
   - Module 4: Sim-to-Real Transfer & Deployment
   - Capstone: End-to-End Humanoid Project
   - **UI Features**: Vibrant gradient themes, animated cards, responsive design, dark mode support

2. **💬 AI-Powered RAG Chatbot**: FastAPI backend with state-of-the-art features:
   - Qdrant vector database for curriculum content indexing
   - Personalized learning paths based on user expertise
   - Urdu translation support for broader accessibility
   - Real-time Q&A via WebSocket streaming
   - **Chat Widget**: Floating action button with pulsing animation, gradient bubbles, smooth transitions

3. **🏗️ Production Infrastructure**:
   - GitHub Actions CI/CD pipelines
   - Comprehensive testing & monitoring
   - Environment-based configuration management

## 🚀 Quick Start

### Prerequisites

- **System**: Still Window soon will switch to Ubuntu (WSL linux system)
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

Visit http://localhost:3000 and sign up with: (Adding soon)
- Email: `test@example.com`
- Password: Your secure password
- Hardware Background: Select your experience level
- Software Background: Select your coding experience

### 2. Ask the Chatbot a Question

Try these sample questions:
- "How do I install ROS 2 Humble?"
- "What is the difference between MoveIt and Gazebo?"
- "Walk me through the sim-to-real pipeline for a robotic arm"

### 3. Enable Personalization (Adding soon)

- Click the "Personalize" toggle in the chatbot widget
- The chatbot will adapt explanations to your experience level

### 4. Try Urdu Translation (Adding soon)

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

## 🐳 Docker Deployment (Adding soon)

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
- **Personalization Latency**: <500ms overhead (soon)
- **Translation Latency**: <1 second per chapter (soon)
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
- **Database**: Encrypted connections with Neon PostgreSQL (soon)
- **Authentication**: JWT tokens via Better-Auth (soon)
- **Password Hashing**: bcrypt with 12+ salt rounds (soon)
- **API Rate Limiting**: 100 requests/min per user
- **CORS**: Restricted to allowed origins (soon)
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

## 📝 Code Documentation

### Backend Code Structure

#### Main Application (`backend/api.py`)
```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Initialize FastAPI app with CORS, logging, and error handling
app = FastAPI(
    title="Physical AI Textbook API",
    description="RAG chatbot for embodied AI curriculum",
    version="1.0.0"
)

# CORS configuration for frontend access
app.add_middleware(CORSMiddleware, ...)

# Health check endpoint
@app.get("/health")
async def health_check():
    """Returns system health status"""
    return {"status": "healthy", "uptime_seconds": ...}

# Chat endpoint (main RAG interface)
@app.post("/api/chat")
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Process user query and return AI response with sources

    Args:
        request: ChatRequest with query string

    Returns:
        ChatResponse with answer, sources, confidence, latency
    """
    # 1. Validate input (5-2000 chars)
    # 2. Retrieve relevant curriculum chunks from Qdrant
    # 3. Call OpenAI Agents with context
    # 4. Format response with citations
```

#### Models & Schemas (`backend/models.py`)
```python
from pydantic import BaseModel, Field

class ChatRequest(BaseModel):
    """User query for chatbot"""
    query: str = Field(..., min_length=5, max_length=2000, description="User question")
    session_id: Optional[str] = Field(None, description="Session identifier")

class ChatResponse(BaseModel):
    """Chatbot response with metadata"""
    answer: str = Field(..., description="AI-generated answer")
    sources: List[str] = Field(default_factory=list, description="Source references")
    confidence: float = Field(..., ge=0, le=1, description="Response confidence score")
    latency_ms: int = Field(..., ge=0, description="Response time in milliseconds")
```

#### RAG Agent (`backend/rag_agent.py`)
```python
from openai import OpenAI

class RAGAgent:
    """
    Retrieval-Augmented Generation agent using OpenAI Agents SDK

    Workflow:
    1. User query → Retrieved from vector database
    2. Relevant chunks → Passed as context to LLM
    3. OpenAI Agents → Generate structured response
    4. Format response → Return with sources and confidence
    """

    def __init__(self, api_key: str, qdrant_client):
        self.client = OpenAI(api_key=api_key)
        self.qdrant = qdrant_client

    def query(self, question: str) -> dict:
        """
        Process user question through RAG pipeline

        Args:
            question: User's natural language question

        Returns:
            dict with answer, sources, confidence, latency
        """
        # 1. Retrieve top-5 relevant chunks from Qdrant
        chunks = self.qdrant.search(question, limit=5)

        # 2. Format context with metadata
        context = self._format_context(chunks)

        # 3. Call OpenAI Agents with tools and system prompt
        response = self._call_agent(question, context)

        # 4. Extract and format response
        return self._parse_response(response, chunks)
```

#### Qdrant Vector Search (`backend/services/qdrant.py`)
```python
from qdrant_client import QdrantClient

class QdrantService:
    """
    Vector database client for curriculum retrieval

    Collection: curriculum_embeddings
    - Vector dimension: 1536 (OpenAI embeddings)
    - Metadata: module, topic, difficulty, source_url
    """

    def __init__(self, url: str, api_key: str):
        self.client = QdrantClient(url, api_key=api_key)

    def search(self, query: str, limit: int = 5) -> list:
        """
        Semantic search for relevant curriculum chunks

        Args:
            query: Natural language search query
            limit: Number of results to return

        Returns:
            List of relevant chunks with metadata
        """
        # 1. Generate embedding for query using OpenAI
        query_embedding = generate_embedding(query)

        # 2. Search Qdrant collection
        results = self.client.search(
            collection_name="curriculum_embeddings",
            query_vector=query_embedding,
            limit=limit
        )

        # 3. Format results with metadata
        return self._format_results(results)
```

### Frontend Code Structure

#### Chat Widget Component (`docosaurus/src/components/ChatWidget/index.tsx`)
```typescript
import React, { useState, useRef, useEffect } from 'react';
import './styles.css';

/**
 * ChatWidget Component
 * Floating chat interface for RAG chatbot interaction
 *
 * Features:
 * - Floating Action Button (FAB) with pulse animation
 * - Message list with user/bot distinction
 * - Auto-scrolling and auto-focus
 * - Real-time API communication
 * - Error handling and loading states
 */
interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  citations?: string[];
  timestamp: number;
}

interface ChatResponse {
  answer: string;
  sources?: string[];
  confidence?: number;
  latency_ms?: number;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([/* initial message */]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    // 1. Validate input
    if (!input.trim()) return;

    // 2. Add user message to UI
    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      text: input,
      sender: 'user',
      timestamp: Date.now(),
    };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // 3. Call backend API
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: input }),
      });

      const data: ChatResponse = await response.json();

      // 4. Add bot response with citations
      const botMessage: Message = {
        id: `msg-${Date.now()}`,
        text: data.answer,
        sender: 'bot',
        citations: data.sources,
        timestamp: Date.now(),
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (err) {
      // Handle and display error
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* FAB Button */}
      <button
        className="chat-widget-fab"
        onClick={() => setIsOpen(!isOpen)}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className="chat-widget-window">
          {/* Header */}
          <div className="chat-widget-header">
            <h3>AI Course Assistant</h3>
          </div>

          {/* Messages List */}
          <div className="chat-widget-messages">
            {messages.map(msg => (
              <div key={msg.id} className={`message ${msg.sender}`}>
                <div className="bubble">{msg.text}</div>
                {msg.citations && (
                  <div className="citations">
                    {msg.citations.map((c, i) => (
                      <a key={i} href={c}>{`Source ${i + 1}`}</a>
                    ))}
                  </div>
                )}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className="input-area">
            <textarea
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyPress={e => e.key === 'Enter' && sendMessage()}
              placeholder="Ask me anything..."
            />
            <button onClick={sendMessage} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
```

#### Global Styles (`docosaurus/src/css/custom.css`)
```css
/**
 * Global Theme Configuration
 *
 * Colors:
 * - Primary: #10b981 (emerald green)
 * - Secondary: #8b5cf6 (purple)
 * - Dark mode: Optimized for accessibility
 *
 * Typography:
 * - Font: Inter (sans-serif)
 * - Size base: 16px
 * - Line height: 1.7
 *
 * Animations:
 * - Transitions: 0.3s-0.4s ease
 * - Easing: cubic-bezier, ease-out
 * - Effects: gradients, shadows, transforms
 */

:root {
  /* Color palette */
  --ifm-color-primary: #10b981;
  --ifm-color-secondary: #8b5cf6;

  /* Typography */
  --ifm-font-family-base: 'Inter', sans-serif;
  --ifm-font-size-base: 16px;
  --ifm-line-height-base: 1.7;

  /* Spacing */
  --ifm-spacing-multiplier: 1.2;
}

/* Component Styles */
.navbar { background: linear-gradient(...); }
.footer { background: linear-gradient(...); }
.chat-widget-fab { animation: pulse-gradient 2s infinite; }
.module-card { transition: all 0.4s cubic-bezier(...); }
```

#### Home Page Features (`docosaurus/src/components/HomepageFeatures/`)
```typescript
/**
 * HomepageFeatures Component
 *
 * Displays:
 * - 6 Module cards with staggered animations
 * - 3 Feature boxes with bottom accent animation
 * - Gradient text effects on titles
 * - Responsive grid layout
 *
 * Animations:
 * - Module cards: fadeInUp (0.1s-0.6s staggered)
 * - Icons: scale(1.1) + rotateY(180deg) on hover
 * - Feature boxes: scaleX bottom border on hover
 */

const ModuleCard = ({ icon, title, subtitle, description }) => (
  <div className={styles.moduleCard}>
    <div className={styles.moduleIcon}>{icon}</div>
    <h3 className={styles.moduleTitle}>{title}</h3>
    <p className={styles.moduleSubtitle}>{subtitle}</p>
    <p className={styles.moduleDescription}>{description}</p>
  </div>
);

const FeatureBox = ({ icon, title, description }) => (
  <div className={styles.featureBox}>
    <div className={styles.featureIcon}>{icon}</div>
    <h3>{title}</h3>
    <p>{description}</p>
  </div>
);
```

## 📝 Contributing

### Code Standards

- **Python**: PEP 8, Google-style docstrings, type hints
- **TypeScript**: ESLint + Prettier
- **Git Commits**: Conventional commits (feat:, fix:, docs:, etc.)
- **Tests**: All PRs must include tests
- **Documentation**: Update docs for new features

### Example Python Docstring
```python
def query_rag_agent(question: str, context: str) -> dict:
    """
    Query the RAG agent with context

    Args:
        question: User's natural language question
        context: Relevant curriculum context

    Returns:
        dict: {
            'answer': str,
            'sources': list[str],
            'confidence': float,
            'latency_ms': int
        }

    Raises:
        ValueError: If question is empty
        TimeoutError: If response takes >30 seconds
    """
    pass
```

### Example TypeScript JSDoc
```typescript
/**
 * Sends a message to the chatbot API
 *
 * @param message - The user's message content
 * @param options - Optional configuration
 * @returns Promise resolving to the bot's response
 * @throws Error if API call fails
 *
 * @example
 * const response = await sendMessage("How do I install ROS 2?");
 * console.log(response.answer);
 */
async function sendMessage(
  message: string,
  options?: SendOptions
): Promise<ChatResponse> {
  // Implementation
}
```

## 🏗️ Project Architecture

### System Overview
```
User (Browser)
    ↓
Frontend (Docosaurus + React)
├── Chat Widget (Floating UI)
├── Homepage (Features, Modules)
├── Curriculum Pages (Markdown content)
└── Dark Mode Support
    ↓ REST/WebSocket API
Backend (FastAPI + Python)
├── Chat Endpoint (/api/chat)
├── RAG Agent (OpenAI Agents SDK)
├── Vector Search (Qdrant Client)
├── Error Handling & Logging
└── Health Check
    ↓ Vector/SQL Queries
Data Layer
├── Qdrant (curriculum_embeddings)
└── PostgreSQL (optional, user profiles)
```

### Technology Stack
| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **Frontend** | Docosaurus | 3.9.2 | Documentation site generator |
| | React | 19.0.0 | UI component framework |
| | TypeScript | 5.6.2 | Type-safe JavaScript |
| | TailwindCSS | 3.4.1 | Utility-first CSS |
| **Backend** | FastAPI | 0.104+ | Python web framework |
| | Python | 3.10+ | Core runtime |
| | OpenAI SDK | latest | Agents SDK integration |
| | Qdrant SDK | latest | Vector database client |
| **Data** | Qdrant | Cloud | Vector embeddings storage |
| | PostgreSQL | latest | Optional, for user data |
| **DevOps** | Docker | latest | Containerization |
| | GitHub Actions | native | CI/CD automation |

### Key Features

#### 1. **Interactive Curriculum**
- 13-week course with 50+ lessons
- ROS 2, Gazebo, Isaac Sim, humanoid robotics
- Copy-paste-run code examples
- Video demonstrations included

#### 2. **AI-Powered Chat Assistant**
- RAG (Retrieval-Augmented Generation) pipeline
- OpenAI Agents SDK for intelligent responses
- Vector search for relevant curriculum chunks
- Real-time response streaming
- Source citations for transparency
- Confidence scoring

#### 3. **Modern UI/UX**
- Vibrant emerald green color scheme
- Gradient effects and smooth animations
- Floating action button for chat
- Dark mode with WCAG AA compliance
- Fully responsive design
- Smooth transitions and hover effects

#### 4. **Developer Experience**
- Type-safe Python with Pydantic
- TypeScript for frontend components
- Comprehensive error handling
- Request/response logging
- Interactive API docs (Swagger)
- Docker for reproducibility

## 📦 Key Files Explained

### Backend Files

**`backend/api.py`**
- FastAPI application entry point
- CORS configuration for frontend access
- Route mounting at `/api`
- Health check endpoint
- Error handling middleware

**`backend/models.py`**
- Pydantic models for request/response validation
- ChatRequest: user's query
- ChatResponse: bot's answer with metadata
- Type hints and field validation

**`backend/router.py`**
- POST /api/chat endpoint definition
- Input validation (query length)
- RAG agent invocation
- Response formatting

**`backend/rag_agent.py`**
- RAGAgent class implementing retrieval-augmented generation
- OpenAI Agents SDK integration
- Query processing workflow
- Response parsing and formatting

**`backend/config.py`**
- Environment variable configuration
- Pydantic Settings for type-safe config
- API keys, URLs, timeouts
- Logging settings

**`backend/data_retrieve.py`**
- Content retrieval and preprocessing
- Embedding generation
- Metadata extraction

**`backend/embeding_helpers.py`**
- Helper functions for embedding operations
- Vector normalization
- Similarity calculations

### Frontend Files

**`docosaurus/docusaurus.config.ts`**
- Docosaurus configuration
- Theme settings
- Sidebar structure
- Navbar items
- Footer links

**`docosaurus/src/css/custom.css`**
- Global color variables (1,136 lines)
- Navbar gradient styling
- Footer animated links
- Hero section backgrounds
- Component transitions
- Dark mode overrides

**`docosaurus/src/components/ChatWidget/index.tsx`**
- Main chat widget component
- Message state management
- API communication
- Auto-scrolling and focus
- Error handling
- Typing indicators

**`docosaurus/src/components/ChatWidget/styles.css`**
- FAB button styling (pulse animation)
- Chat window styling (bouncy entrance)
- Message bubble gradients
- Input area styling
- Responsive mobile adjustments

**`docosaurus/src/components/HomepageFeatures/index.tsx`**
- Module cards display
- Feature boxes
- Responsive grid layout
- Icon rendering

**`docosaurus/src/components/HomepageFeatures/styles.module.css`**
- Module card styling (1,136 lines enhanced)
- Feature box styling
- Staggered animations
- Hover effects with transforms
- Gradient text effects

## 🎓 Curriculum Structure

### Module Organization
```
Module 0: Foundations
├── Physical AI concepts
├── Hardware vs Software
└── Course overview

Module 1: ROS 2 (Weeks 3-5)
├── ROS 2 Architecture
├── Nodes, Topics, Services
├── Gazebo Simulation
└── Practical projects

Module 2: Control Systems (Weeks 6-7)
├── Motion Planning
├── Control Theory
├── Trajectory Optimization
└── Real-world applications

Module 3: Isaac Sim (Weeks 8-10)
├── Digital Twins
├── Sim-to-Real Transfer
├── Isaac Sim environment
└── Deployment strategies

Module 4: VLA & Humanoids (Weeks 11-13)
├── Vision-Language-Action
├── Humanoid Kinematics
├── Humanoid Dynamics
└── Conversational Robotics

Capstone: End-to-End Project
└── Full pipeline from simulation to real hardware
```

## 🐛 Troubleshooting

### Build Error: npm run build

**Solution Steps:**
1. Clear cache: `rm -rf .docusaurus node_modules`
2. Reinstall deps: `npm install`
3. Run build: `npm run build`
4. If error persists, check for:
   - TypeScript compilation errors
   - Missing CSS imports
   - Circular dependencies in components

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
- **Email**: ayeshamughal2513@gmail.com

## 🙏 Acknowledgments

- **ROS 2 Community** for foundational robotics frameworks
- **NVIDIA** for Isaac Sim and robotics resources
- **OpenAI** for GPT models and Agents SDK
- **Qdrant** for vector database infrastructure
- **Neon** for managed PostgreSQL

## 🎨 Recent UI/UX Enhancements (December 18, 2025)

### Global Styling
- ✅ Vibrant emerald green primary color (#10b981) with purple accents
- ✅ Gradient backgrounds and text effects throughout
- ✅ Enhanced typography with Inter font for readability
- ✅ Comprehensive dark mode support with proper contrast ratios

### Navigation & Footer
- ✅ Gradient navbar with animated underlines on links
- ✅ Enhanced footer with animated bullet indicators
- ✅ Smooth transitions on all interactive elements

### Chat Widget
- ✅ Pulsing gradient FAB button with enhanced shadows
- ✅ Bouncy entrance animation with rounded corners
- ✅ Gradient message bubbles with smooth transitions
- ✅ Improved input area with better focus states

### Homepage Features
- ✅ Staggered fade-in animations for cards
- ✅ Gradient text on titles and subtitles
- ✅ Icon scale and 3D rotation effects on hover
- ✅ Responsive design for all screen sizes

**Total CSS Enhancements**: 1,136 lines 

## 🗺️ Roadmap

### Phase 1 : Foundation ✅
- [x] Project setup & CI/CD
- [x] FastAPI + Docosaurus skeleton
- [x] Database provisioning
- [x] UI/UX enhancements with modern styling

### Phase 2 : Infrastructure
- [ ] Better-Auth integration (50 pts)
- [ ] Personalization engine (50 pts)
- [ ] Urdu translation service (50 pts)
- [ ] Comprehensive documentation

### Phase 3 : Core Features
- [ ] RAG chatbot implementation
- [ ] Module 1 content & demos

### Phase 4 : Completion & Launch
- [ ] Full curriculum content
- [ ] Pre-launch testing
- [ ] Production deployment



## 📊 Metrics

Track platform metrics in real-time:
- **UI Performance**: Docosaurus Lighthouse score ≥96



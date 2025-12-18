# Contributing to Physical AI Textbook Platform

Thank you for your interest in contributing! This guide will help you understand how to contribute to this project effectively.

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Getting Started](#getting-started)
3. [Development Setup](#development-setup)
4. [Commit Message Guidelines](#commit-message-guidelines)
5. [Pull Request Process](#pull-request-process)
6. [Code Style Guidelines](#code-style-guidelines)
7. [Testing Requirements](#testing-requirements)
8. [Documentation Guidelines](#documentation-guidelines)
9. [Issue Guidelines](#issue-guidelines)

## Code of Conduct

We are committed to providing a welcoming and inclusive environment for all contributors. Please:

- Be respectful and inclusive in all interactions
- Welcome people of all backgrounds and expertise levels
- Ask questions and seek clarification
- Give credit where due
- Focus on constructive criticism

## Getting Started

### Prerequisites

Before you start, ensure you have:

- **Git**: Latest version
- **Python**: 3.10+
- **Node.js**: 18+
- **Docker & Docker Compose**: Latest
- **Poetry** (optional, for Python dependency management)
- **pre-commit**: For Git hooks

### Fork and Clone

```bash
# Fork the repository on GitHub

# Clone your fork
git clone https://github.com/YOUR-USERNAME/Humanoid-Robotic-Course-book.git
cd Humanoid-Robotic-Course-book

# Add upstream remote
git remote add upstream https://github.com/ayesha-offical/Humanoid-Robotic-Course-book.git

# Create a feature branch
git checkout -b feature/your-feature-name
```

## Development Setup

### 1. Install Pre-commit Hooks

```bash
pip install pre-commit
pre-commit install

# Run hooks on all files
pre-commit run --all-files
```

### 2. Backend Development

```bash
cd backend

# Install dependencies
pip install -r requirements.txt
# OR using Poetry
poetry install

# Setup environment
cp ../.env.example ../.env
# Edit .env with your configuration

# Run migrations
alembic upgrade head

# Start development server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Frontend Development

```bash
cd docosaurus

# Install dependencies
npm install

# Copy environment template
cp ../.env.example .env.local

# Start development server
npm start
```

### 4. Run Tests

```bash
cd backend

# Run all tests
pytest tests/

# Run with coverage
pytest tests/ --cov=src --cov-report=html

# Run specific test file
pytest tests/integration/test_auth_login.py -v
```

## Commit Message Guidelines

We follow [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type

- **feat**: New feature
- **fix**: Bug fix
- **docs**: Documentation changes
- **style**: Code style changes (formatting, semicolons, etc.)
- **refactor**: Code refactoring without functional changes
- **perf**: Performance improvements
- **test**: Adding or updating tests
- **chore**: Maintenance tasks, dependency updates
- **ci**: CI/CD changes
- **build**: Build system changes

### Scope (Optional)

Scope specifies the area affected:

- `auth`: Authentication changes
- `chatbot`: Chatbot/RAG changes
- `personalization`: Personalization feature
- `translation`: Translation service
- `frontend`: Frontend/Docosaurus
- `backend`: Backend/FastAPI
- `database`: Database/migrations
- `docker`: Docker configuration
- `ci`: CI/CD pipelines

### Subject

- Imperative mood: "add feature" not "added feature"
- Don't capitalize first letter
- No period at end
- Keep under 50 characters

### Body (Optional)

- Explain what and why, not how
- Wrap at 72 characters
- Separate from subject with blank line

### Footer (Optional)

Reference issues and breaking changes:

```
Closes #123
Fixes #456
BREAKING CHANGE: describe the breaking change
```

### Examples

```
feat(auth): implement Better-Auth signup with background capture

Add hardware_background and software_background fields to user signup form.
Integrate Better-Auth for secure password hashing and JWT token generation.

Closes #45
```

```
fix(chatbot): resolve WebSocket timeout issue in streaming

Increase timeout from 30s to 60s and add connection keep-alive pings.
Also improve error handling for disconnected clients.

Fixes #123
```

```
docs: update README with quick-start instructions
```

## Pull Request Process

### Before You Start

1. Check existing issues/PRs to avoid duplicate work
2. Create an issue to discuss major changes first
3. Keep PRs focused on a single feature or fix
4. Update branch before creating PR: `git pull upstream main`

### Creating Your PR

1. **Push to your fork**:
   ```bash
   git push origin feature/your-feature-name
   ```

2. **Create PR on GitHub**:
   - Title: Follow commit message format (feat/fix/etc)
   - Description: Use the template below
   - Link related issues: "Closes #123"

### PR Template

```markdown
## Description
Brief description of changes.

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Related Issues
Closes #123

## How to Test
1. Step one
2. Step two
3. Expected result

## Checklist
- [ ] Code follows style guidelines
- [ ] Tests added/updated
- [ ] Documentation updated
- [ ] No new warnings generated
- [ ] Self-reviewed changes
- [ ] Comments added for complex logic

## Performance Impact
- [ ] No impact
- [ ] Improved
- [ ] Degraded (explain why acceptable)

## Additional Context
Any additional information.
```

### Review & Feedback

- Respond to feedback promptly
- Make requested changes in new commits
- Ask for clarification if needed
- Be patient - reviews take time

### Merge Criteria

Your PR can be merged when:
- ✅ All CI/CD checks pass
- ✅ At least one approval from maintainer
- ✅ All feedback addressed
- ✅ Tests passing with good coverage
- ✅ Documentation updated
- ✅ No merge conflicts

## Code Style Guidelines

### Python

```bash
# Run formatters
black backend/
isort backend/

# Run linters
flake8 backend/
mypy backend/

# Or use pre-commit
pre-commit run --all-files
```

**Standards**:
- Follow [PEP 8](https://pep8.org/)
- Use 4 spaces for indentation
- Line length: 100 characters
- Type hints on public functions
- Google-style docstrings

**Example**:

```python
def get_user_profile(user_id: str) -> UserResponse:
    """
    Retrieve user profile by ID.

    Args:
        user_id: The unique user identifier.

    Returns:
        UserResponse: User profile data.

    Raises:
        UserNotFoundError: If user doesn't exist.
    """
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise UserNotFoundError(f"User {user_id} not found")
    return UserResponse.from_orm(user)
```

### TypeScript/JavaScript

```bash
# Run formatters
prettier --write docosaurus/

# Run linter
eslint docosaurus/src/ --fix

# Or use pre-commit
pre-commit run --all-files
```

**Standards**:
- Follow ESLint configuration
- Use Prettier for formatting
- 2 spaces for indentation
- Use TypeScript for type safety
- Document complex components

**Example**:

```typescript
interface ChatbotWidgetProps {
  userId?: string;
  onClose?: () => void;
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  userId,
  onClose,
}) => {
  const [messages, setMessages] = React.useState<Message[]>([]);

  // Component implementation
  return <div>{/* JSX */}</div>;
};
```

### Markdown

- Use 2 spaces for indentation
- Order headings logically (h1, h2, h3)
- Wrap text at 80 characters when possible
- Use descriptive link text: `[link text](url)` not `[click here](url)`
- Code blocks with language: ` ```python `

## Testing Requirements

### Test Coverage

- **Minimum**: 80% code coverage
- **Target**: 90%+ for critical paths
- Focus on unit tests for logic
- Integration tests for APIs
- Performance tests for latency-sensitive code

### Writing Tests

```python
import pytest
from fastapi.testclient import TestClient

def test_signup_success(client: TestClient, db: Session):
    """Test successful user signup."""
    response = client.post(
        "/api/auth/signup",
        json={
            "email": "test@example.com",
            "password": "SecurePassword123!",
            "hardware_background": "beginner",
            "software_background": "some_python",
        },
    )

    assert response.status_code == 201
    assert response.json()["email"] == "test@example.com"

    # Verify user in database
    user = db.query(User).filter(User.email == "test@example.com").first()
    assert user is not None
```

### Running Tests

```bash
cd backend

# Run all tests
pytest tests/

# Run specific test
pytest tests/integration/test_auth_login.py::test_login_success -v

# Run with coverage
pytest tests/ --cov=src --cov-report=html

# Run in parallel
pytest tests/ -n auto

# Run only fast tests
pytest tests/ -m "not slow"
```

## Documentation Guidelines

### Docstrings

Use Google-style docstrings:

```python
def complex_function(param1: str, param2: int) -> dict:
    """
    Brief one-line description.

    More detailed explanation of what the function does,
    including any important behavior or side effects.

    Args:
        param1: Description of param1.
        param2: Description of param2.

    Returns:
        dict: Description of return value with structure.

    Raises:
        ValueError: When input is invalid.

    Examples:
        >>> complex_function("test", 42)
        {'result': 'success'}
    """
    pass
```

### README Changes

- Update if adding new features
- Include examples and setup instructions
- Keep quick-start section up to date

### API Documentation

- Document all endpoints in docstrings
- Use OpenAPI/Swagger format
- Include request/response examples

### Commit Changes

Document breaking changes:

```
BREAKING CHANGE: Remove deprecated `/api/old-endpoint` in favor of `/api/new-endpoint`
```

## Issue Guidelines

### Creating Issues

**Bug Reports**:
```markdown
## Description
Brief description of the bug.

## Steps to Reproduce
1. Step 1
2. Step 2
3. Result

## Expected Behavior
What should happen.

## Actual Behavior
What actually happens.

## Environment
- OS: Ubuntu 22.04
- Python: 3.10
- Browser: Chrome 120
```

**Feature Requests**:
```markdown
## Description
Brief description of the feature.

## Motivation
Why is this needed?

## Proposed Solution
How should this work?

## Alternatives Considered
Any other approaches?
```

### Issue Labels

- `bug`: Something broken
- `feature`: New functionality
- `docs`: Documentation
- `help wanted`: Looking for contributors
- `good first issue`: Good for newcomers
- `backend`: Backend/FastAPI
- `frontend`: Frontend/Docosaurus
- `urgent`: Needs immediate attention

## Getting Help

- **Discussions**: [GitHub Discussions](https://github.com/ayesha-offical/Humanoid-Robotic-Course-book/discussions)
- **Issues**: [GitHub Issues](https://github.com/ayesha-offical/Humanoid-Robotic-Course-book/issues)
- **Email**: support@robotics-textbook.org

## Recognition

Contributors will be recognized:
- In README acknowledgments
- In release notes
- On project website

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing to making robotics education more accessible! 🤖

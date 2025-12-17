# Chatbot Integration into Docosaurus Website

## Overview
The Physical AI Assistant chatbot has been integrated into your Docosaurus website. It appears as a floating button in the bottom-right corner of every page and supports text selection context.

## Implementation Summary

### 1. Updated Components

#### useChat Hook (docusaurus/src/hooks/useChat.ts)
- Updated to support selected_text parameter
- Now sends requests to backend with format:
  - message: user query
  - selected_text: highlighted text or null
  - user_id: guest_user
  - session_id: optional
  - personalize: true/false

### 2. New Chat Widget Component
- File: docosaurus/src/components/ChatWidget.tsx (new component)
- Features:
  - Floating chat button (bottom-right)
  - Expanded chat window when clicked
  - Message history with styling
  - Loading spinner during API calls
  - Error handling
  - TEXT SELECTION: Captures text highlighted by users on the page
  - Displays "Context" indicator showing selected text
  - Automatically sends selected text with messages

### 3. Global Integration via Root Component
- File: docosaurus/src/theme/Root.tsx (new file)
- Renders ChatWidget on every page
- Docosaurus automatically uses this file to wrap all pages
- Provides auth token from useAuth hook

## Backend API Integration

POST http://localhost:8000/api/ask

Request Body:
{
  "message": "What is ROS 2?",
  "selected_text": "ROS 2 is...",
  "user_id": "guest_user",
  "session_id": null,
  "personalize": true
}

Response:
{
  "answer": "ROS 2 is a robotics middleware platform...",
  "sources": [{"title": "Lesson 1", "url": "..."}],
  "confidence": 0.95,
  "session_id": "session-123",
  "latency_ms": 245
}

## File Structure

docusaurus/
├── src/
│   ├── components/
│   │   ├── ChatWidget.tsx          (NEW)
│   │   ├── ChatbotWidget.tsx       (DEPRECATED)
│   │   ├── AuthModal.tsx
│   │   ├── PersonalizeButton.tsx
│   │   └── LanguageSwitcher.tsx
│   ├── hooks/
│   │   ├── useChat.ts             (UPDATED)
│   │   ├── useAuth.ts
│   │   ├── usePersonalization.ts
│   │   └── useTranslation.ts
│   ├── theme/
│   │   └── Root.tsx               (NEW)
│   ├── css/
│   └── pages/
├── docusaurus.config.ts           (NO CHANGES)
└── package.json

## Usage

### For Users
1. Click the floating "Chat" button in the bottom-right corner
2. Highlight text anywhere on the page while the chat is open
3. Type a question and click Send
4. Selected text appears in an amber-colored box below the header
5. Sign in via AuthModal to send messages

### For Developers

Customizing the ChatWidget:
- Edit docosaurus/src/components/ChatWidget.tsx
- Change styling (Tailwind classes)
- Adjust floating position (currently bottom-6 right-6)
- Modify messages and labels

## Environment Setup

Frontend (.env.local):
REACT_APP_API_URL=http://localhost:8000

Backend Requirements:
- Handle POST /api/ask endpoint
- Accept: message, selected_text, user_id, session_id, personalize
- Return: answer, sources, confidence, session_id, latency_ms

## Styling

ChatWidget uses Tailwind CSS classes:
- Header: bg-blue-600, hover: bg-blue-700
- User messages: bg-blue-600 text-white
- Bot messages: bg-gray-200 text-gray-900
- Context indicator: bg-amber-50
- Error messages: text-red-500

Position:
- Fixed: bottom-6 right-6
- Z-index: z-40 (button), z-50 (window)
- Width: w-96 with max-w-[90vw] for mobile

## Text Selection Feature

How It Works:
1. When ChatWidget is open, event listeners monitor mouse and touch events
2. On mouseup/touchend, window.getSelection().toString() captures selected text
3. Selected text is stored and displayed in context indicator
4. Sent with API request, then cleared after message

Use Cases:
- "Explain this in simpler terms"
- "How does this relate to ROS 2?"
- "Why doesn't this code work?"
- "What are the prerequisites for this?"

## Files Modified/Created

Created:
- docosaurus/src/components/ChatWidget.tsx
- docosaurus/src/theme/Root.tsx
- CHATBOT_INTEGRATION.md (this file)

Modified:
- docosaurus/src/hooks/useChat.ts (added selectedText parameter)

No Changes:
- docosaurus/docusaurus.config.ts (auto-uses Root.tsx)
- Other components unchanged

---
Last Updated: 2025-12-11
Docosaurus Version: 3.x
React Version: 18+

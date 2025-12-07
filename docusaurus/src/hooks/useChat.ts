import { useState, useCallback } from 'react';

export interface Message {
  role: 'user' | 'assistant';
  content: string;
}

export interface ChatResponse {
  answer: string;
  sources: Array<{ title: string; url: string }>;
  confidence: number;
  session_id: string;
  latency_ms: number;
}

interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sessionId: string | null;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const useChat = (token: string | null) => {
  const [state, setState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    error: null,
    sessionId: null,
  });

  const chat = useCallback(
    async (
      query: string,
      sessionId?: string | null,
      personalize: boolean = false
    ): Promise<ChatResponse | null> => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        if (!token) {
          throw new Error('Authentication required');
        }

        const response = await fetch(`${API_URL}/api/ask`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query,
            session_id: sessionId || undefined,
            personalize,
          }),
        });

        if (!response.ok) {
          throw new Error('Chat request failed');
        }

        const data: ChatResponse = await response.json();

        setState((prev) => ({
          ...prev,
          messages: [
            ...prev.messages,
            { role: 'user', content: query },
            { role: 'assistant', content: data.answer },
          ],
          sessionId: data.session_id,
          isLoading: false,
        }));

        return data;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Chat error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return null;
      }
    },
    [token]
  );

  const streamChat = useCallback(
    async (query: string, sessionId?: string | null) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        if (!token) {
          throw new Error('Authentication required');
        }

        const ws = new WebSocket(
          `${API_URL.replace('http', 'ws')}/ws/chat`
        );

        return new Promise<void>((resolve, reject) => {
          let fullResponse = '';

          ws.onopen = () => {
            ws.send(
              JSON.stringify({
                query,
                session_id: sessionId || undefined,
              })
            );
          };

          ws.onmessage = (event) => {
            const chunk = JSON.parse(event.data);
            if (chunk.token) {
              fullResponse += chunk.token;
              setState((prev) => {
                const messages = [...prev.messages];
                if (
                  messages.length === 0 ||
                  messages[messages.length - 1].role === 'user'
                ) {
                  messages.push({
                    role: 'assistant',
                    content: chunk.token,
                  });
                } else {
                  messages[messages.length - 1].content += chunk.token;
                }
                return { ...prev, messages };
              });
            }
            if (chunk.done) {
              setState((prev) => ({
                ...prev,
                sessionId: chunk.session_id,
                isLoading: false,
              }));
              resolve();
              ws.close();
            }
          };

          ws.onerror = (error) => {
            const errorMessage = 'WebSocket error';
            setState((prev) => ({
              ...prev,
              isLoading: false,
              error: errorMessage,
            }));
            reject(error);
          };
        });
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Stream error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        throw error;
      }
    },
    [token]
  );

  const clearChat = useCallback(() => {
    setState({
      messages: [],
      isLoading: false,
      error: null,
      sessionId: null,
    });
  }, []);

  return {
    ...state,
    chat,
    streamChat,
    clearChat,
  };
};

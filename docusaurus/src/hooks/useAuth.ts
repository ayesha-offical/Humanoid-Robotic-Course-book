import { useState, useCallback } from 'react';

interface User {
  id: string;
  email: string;
  hardware_background: string;
  software_background: string;
  created_at: string;
}

interface AuthState {
  user: User | null;
  token: string | null;
  isLoading: boolean;
  error: string | null;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const useAuth = () => {
  const [state, setState] = useState<AuthState>({
    user: null,
    token: localStorage.getItem('access_token'),
    isLoading: false,
    error: null,
  });

  const signup = useCallback(
    async (
      email: string,
      password: string,
      hardware_background: string,
      software_background: string
    ) => {
      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const response = await fetch(`${API_URL}/api/auth/signup`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email,
            password,
            hardware_background,
            software_background,
          }),
        });

        if (!response.ok) {
          throw new Error('Signup failed');
        }

        const data = await response.json();
        localStorage.setItem('access_token', data.access_token);
        setState((prev) => ({
          ...prev,
          token: data.access_token,
          user: data.user,
          isLoading: false,
        }));
        return data;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Signup error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        throw error;
      }
    },
    []
  );

  const login = useCallback(async (email: string, password: string) => {
    setState((prev) => ({ ...prev, isLoading: true, error: null }));
    try {
      const response = await fetch(`${API_URL}/api/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        throw new Error('Login failed');
      }

      const data = await response.json();
      localStorage.setItem('access_token', data.access_token);
      setState((prev) => ({
        ...prev,
        token: data.access_token,
        user: data.user,
        isLoading: false,
      }));
      return data;
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Login error';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      throw error;
    }
  }, []);

  const logout = useCallback(async () => {
    try {
      if (state.token) {
        await fetch(`${API_URL}/api/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${state.token}`,
            'Content-Type': 'application/json',
          },
        });
      }
      localStorage.removeItem('access_token');
      setState({
        user: null,
        token: null,
        isLoading: false,
        error: null,
      });
    } catch (error) {
      console.error('Logout error:', error);
    }
  }, [state.token]);

  const getCurrentUser = useCallback(async () => {
    if (!state.token) {
      return null;
    }
    try {
      const response = await fetch(`${API_URL}/api/auth/me`, {
        headers: {
          'Authorization': `Bearer ${state.token}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch user');
      }

      const data = await response.json();
      setState((prev) => ({ ...prev, user: data }));
      return data;
    } catch (error) {
      console.error('Error fetching current user:', error);
      return null;
    }
  }, [state.token]);

  return {
    ...state,
    signup,
    login,
    logout,
    getCurrentUser,
  };
};

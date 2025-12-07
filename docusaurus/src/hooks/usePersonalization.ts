import { useState, useCallback } from 'react';

interface PersonalizationProfile {
  user_id: string;
  hardware_background: string;
  software_background: string;
  personalization_enabled: boolean;
  language: string;
  theme: string;
}

interface PersonalizationState {
  profile: PersonalizationProfile | null;
  level: 'beginner' | 'intermediate' | 'advanced' | null;
  isLoading: boolean;
  error: string | null;
}

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const usePersonalization = (token: string | null) => {
  const [state, setState] = useState<PersonalizationState>({
    profile: null,
    level: null,
    isLoading: false,
    error: null,
  });

  const getPersonalizationLevel = useCallback(async () => {
    if (!token) {
      return null;
    }

    setState((prev) => ({ ...prev, isLoading: true, error: null }));
    try {
      const response = await fetch(`${API_URL}/api/personalize/profile`, {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch personalization profile');
      }

      const profile: PersonalizationProfile = await response.json();

      // Determine personalization level based on backgrounds
      let level: 'beginner' | 'intermediate' | 'advanced' = 'beginner';
      if (
        profile.hardware_background.includes('advanced') ||
        profile.software_background.includes('advanced')
      ) {
        level = 'advanced';
      } else if (
        profile.hardware_background.includes('intermediate') ||
        profile.software_background.includes('intermediate')
      ) {
        level = 'intermediate';
      }

      setState((prev) => ({
        ...prev,
        profile,
        level,
        isLoading: false,
      }));

      return { profile, level };
    } catch (error) {
      const errorMessage =
        error instanceof Error ? error.message : 'Error fetching profile';
      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
      return null;
    }
  }, [token]);

  const setPersonalizationEnabled = useCallback(
    async (enabled: boolean) => {
      if (!token) {
        return false;
      }

      setState((prev) => ({ ...prev, isLoading: true, error: null }));
      try {
        const response = await fetch(`${API_URL}/api/personalize/profile`, {
          method: 'PUT',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            personalization_enabled: enabled,
          }),
        });

        if (!response.ok) {
          throw new Error('Failed to update personalization settings');
        }

        const updatedProfile: PersonalizationProfile =
          await response.json();
        setState((prev) => ({
          ...prev,
          profile: updatedProfile,
          isLoading: false,
        }));

        return true;
      } catch (error) {
        const errorMessage =
          error instanceof Error ? error.message : 'Update error';
        setState((prev) => ({
          ...prev,
          isLoading: false,
          error: errorMessage,
        }));
        return false;
      }
    },
    [token]
  );

  return {
    ...state,
    getPersonalizationLevel,
    setPersonalizationEnabled,
  };
};

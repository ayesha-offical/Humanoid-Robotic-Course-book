import React, { useState, useEffect } from 'react';
import { useTranslation } from '../hooks/useTranslation';

interface LanguageSwitcherProps {
  currentLanguage?: string;
  onLanguageChange?: (language: string) => void;
}

export const LanguageSwitcher: React.FC<LanguageSwitcherProps> = ({
  currentLanguage = 'en',
  onLanguageChange,
}) => {
  const { supportedLanguages, getSupportedLanguages, isLoading } =
    useTranslation();
  const [selectedLanguage, setSelectedLanguage] = useState(currentLanguage);

  useEffect(() => {
    getSupportedLanguages();
  }, [getSupportedLanguages]);

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newLanguage = e.target.value;
    setSelectedLanguage(newLanguage);
    onLanguageChange?.(newLanguage);
  };

  if (isLoading || supportedLanguages.length === 0) {
    return (
      <button
        disabled
        className="px-3 py-2 text-sm rounded-lg bg-gray-200 text-gray-600 cursor-not-allowed"
      >
        🌐 Loading...
      </button>
    );
  }

  return (
    <select
      value={selectedLanguage}
      onChange={handleLanguageChange}
      className="px-3 py-2 text-sm rounded-lg bg-white border border-gray-300 hover:border-gray-400 focus:outline-none focus:ring-2 focus:ring-blue-500 cursor-pointer"
      aria-label="Select language"
    >
      {supportedLanguages.map((lang) => (
        <option key={lang.code} value={lang.code}>
          {lang.name}
        </option>
      ))}
    </select>
  );
};

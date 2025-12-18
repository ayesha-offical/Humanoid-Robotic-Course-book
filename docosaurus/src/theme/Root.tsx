import type { ReactNode } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }: { children: ReactNode }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}

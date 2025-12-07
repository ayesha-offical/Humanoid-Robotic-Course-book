import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className={clsx(styles.heroTitle, 'hero-animate')} style={{ animationDelay: '0s' }}>
          Physical AI & Humanoid Robotics Textbook
        </Heading>
        <p className={clsx(styles.heroSubtitle, 'hero-animate')} style={{ animationDelay: '0.2s' }}>
          A complete and practical learning system where you master the future: humanoid robotics, ROS 2, large action models, simulation, VLA systems, hardware, and advanced AI for next-generation intelligent machines.
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx("button", styles.buttonPrimary, 'hero-animate')}
            style={{ animationDelay: '0.4s' }}
            to="/docs/intro">
            Start Reading →
          </Link>
        </div>
      </div>
    </header>
  );
}

function BottomCTA() {
  return (
    <section className={styles.bottomCta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Begin Your Robotics Journey
          </Heading>
          <Link
            className={clsx("button", styles.buttonPrimary)}
            to="/docs/intro">
            Start Reading →
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A complete and practical learning system for humanoid robotics, ROS 2, simulation, and advanced AI">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <BottomCTA />
      </main>
    </Layout>
  );
}

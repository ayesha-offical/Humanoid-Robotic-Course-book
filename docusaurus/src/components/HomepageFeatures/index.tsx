import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  subtitle: string;
  icon: string;
  description: ReactNode;
};

type FeatureColumn = {
  title: string;
  description: ReactNode;
  icon: string;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'ROS 2 Foundations',
    subtitle: 'The Robotic Nervous System',
    icon: '🤖',
    description: (
      <>
        Master ROS 2 architecture, nodes, topics, and services. Build modular robotic applications with Humble distribution.
      </>
    ),
  },
  {
    title: 'Simulation & Digital Twins',
    subtitle: 'Gazebo, Unity, Isaac Sim',
    icon: '🌐',
    description: (
      <>
        Design and simulate complex robotic systems. Perfect your algorithms before real-world deployment.
      </>
    ),
  },
  {
    title: 'Hardware Foundations',
    subtitle: 'Sensors, Actuators, Edge AI',
    icon: '⚙️',
    description: (
      <>
        Integrate hardware components, process sensor data, and deploy edge AI models on robotic platforms.
      </>
    ),
  },
  {
    title: 'VLA - Vision, Language, Action',
    subtitle: 'Bridging Perception & Control',
    icon: '👁️',
    description: (
      <>
        Combine vision, language understanding, and motor control. Build embodied AI systems that understand and interact with the world.
      </>
    ),
  },
  {
    title: 'Advanced AI & Motion Control',
    subtitle: 'Reinforcement Learning',
    icon: '🧠',
    description: (
      <>
        Implement reinforcement learning for robot control. Master trajectory optimization and adaptive behavior.
      </>
    ),
  },
  {
    title: 'Designing Humanoid Robots',
    subtitle: 'Kinematics & Dynamics',
    icon: '🦾',
    description: (
      <>
        Understand humanoid robot design principles. Learn kinematics, dynamics, and whole-body control strategies.
      </>
    ),
  },
];

const FeatureColumns: FeatureColumn[] = [
  {
    title: 'AI-Driven Design',
    icon: '⚡',
    description: (
      <>
        Every lesson is enhanced with AI personalization. The system adapts explanations based on your background—beginners get fundamentals, advanced learners get optimization strategies.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    icon: '💻',
    description: (
      <>
        Copy-paste-run code examples in every lesson. Simulate in Gazebo, test on Jetson devices, deploy to real hardware with confidence.
      </>
    ),
  },
  {
    title: 'Industry-Inspired Curriculum',
    icon: '🎯',
    description: (
      <>
        Built on real-world robotics challenges from NVIDIA, Boston Dynamics, and Tesla. Learn the frameworks and techniques used in production systems.
      </>
    ),
  },
];

function ModuleCard({title, subtitle, icon, description}: ModuleItem) {
  return (
    <div className={clsx('col col--4', styles.moduleCol)}>
      <div className={styles.moduleCard}>
        <div className={styles.moduleIcon}>{icon}</div>
        <Heading as="h3" className={styles.moduleTitle}>{title}</Heading>
        <p className={styles.moduleSubtitle}>{subtitle}</p>
        <p className={styles.moduleDescription}>{description}</p>
      </div>
    </div>
  );
}

function FeatureBox({title, icon, description}: FeatureColumn) {
  return (
    <div className={clsx('col col--4', styles.featureCol)}>
      <div className={styles.featureBox}>
        <div className={styles.featureIcon}>{icon}</div>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      {/* What This Textbook Covers Section */}
      <section className={styles.modules}>
        <div className="container">
          <div className={styles.modulesHeader}>
            <Heading as="h2" className={styles.sectionTitle}>
              What This Textbook Covers
            </Heading>
            <p className={styles.sectionSubtitle}>
              A complete curriculum spanning ROS 2, simulation, hardware integration, vision-language-action systems, and advanced AI.
            </p>
          </div>
          <div className="row">
            {ModuleList.map((props, idx) => (
              <ModuleCard key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>

      {/* Why This Textbook is AI-Native Section */}
      <section className={styles.features}>
        <div className="container">
          <div className={styles.modulesHeader}>
            <Heading as="h2" className={styles.sectionTitle}>
              Why This Textbook is AI-Native
            </Heading>
          </div>
          <div className="row">
            {FeatureColumns.map((props, idx) => (
              <FeatureBox key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>
    </>
  );
}

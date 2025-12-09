import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

const modules = [
  {
    title: 'Robotic Nervous System (ROS 2)',
    description: 'Master the core framework for robot software development, communication, and control.',
    link: '/module1-ros2/ros2-fundamentals',
  },
  {
    title: 'Digital Twin (Gazebo & Unity)',
    description: 'Build and test in high-fidelity simulations before deploying to physical hardware.',
    link: '/module2-digital-twin/digital-twins-in-robotics',
  },
  {
    title: 'AI-Robot Brain (NVIDIA Isaac)',
    description: 'Leverage powerful tools for perception, navigation, and manipulation in AI-driven robotics.',
    link: '/module3-ai-robot-brain/isaac-sim-overview',
  },
  {
    title: 'Vision-Language-Action (VLA + LLMs)',
    description: 'Integrate cutting-edge large language models to enable complex, multi-modal robot behaviors.',
    link: '/module4-vla-capstone/vla-fundamentals',
  },
];

const outcomes = [
    'Physical AI principles and embodied intelligence',
    'ROS 2 for humanoid control',
    'Robot simulation with Gazebo & Unity',
    'AI-powered humanoid behavior with NVIDIA Isaac',
    'Conversational robotics with GPT/LLM integration'
];

function Hero() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <h1 className={clsx('hero__title', styles.heroTitle)}>Physical AI & Humanoid Robotics</h1>
        <p className={clsx('hero__subtitle', styles.heroSubtitle)}>Bridging the Gap Between Digital Intelligence and the Physical World</p>
        <p className={styles.heroDescription}>
          Learn to design, simulate, and control humanoid robots using ROS 2, Gazebo, NVIDIA Isaac, and LLM-powered Vision-Language-Action systems.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/introduction/course-overview">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModulesSection() {
    return (
      <section className={styles.features}>
        <div className="container">
          <h2 className={styles.sectionTitle}>Course Modules</h2>
          <div className={styles.cardContainer}>
            {modules.map((module, idx) => (
              <Link to={module.link} key={idx} className={clsx('card', styles.card)}>
                <div className="card__header">
                  <h3 className={styles.cardTitle}>{module.title}</h3>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                </div>
              </Link>
            ))}
          </div>
        </div>
      </section>
    );
  }

  function OutcomesSection() {
    return (
      <section className={clsx(styles.features, styles.outcomesSection)}>
        <div className="container">
          <h2 className={styles.sectionTitle}>Learning Outcomes</h2>
          <p className={styles.sectionDescription}>
            Upon completing this textbook, you will have the foundational knowledge and practical skills to build and command sophisticated humanoid robots, preparing you for advanced research and development in the field of Physical AI.
          </p>
          <div className={styles.cardContainer}>
            {outcomes.map((outcome, idx) => (
              <div key={idx} className={clsx('card', styles.card, styles.outcomeCard)}>
                 <div className="card__header">
                    <h3 className={styles.cardTitle}>{outcome}</h3>
                 </div>
              </div>
            ))}
          </div>
        </div>
      </section>
    );
  }


function ChatbotCallout() {
    return (
        <section className={styles.chatbotSection}>
            <div className="container">
                <div className={styles.chatbotCallout} onClick={() => { /* In the future, this could open a chat modal */ }}>
                    <p>Ask the AI any question about the textbook content!</p>
                </div>
            </div>
        </section>
    );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Learn to design, simulate, and control humanoid robots using ROS 2, Gazebo, NVIDIA Isaac, and LLM-powered Vision-Language-Action systems.">
      <Hero />
      <main>
        <ModulesSection />
        <OutcomesSection />
        <ChatbotCallout />
      </main>
    </Layout>
  );
}
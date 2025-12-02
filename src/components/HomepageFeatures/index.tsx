import type { ReactNode } from "react";
import clsx from "clsx";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: "Physical AI Foundations",
    description: (
      <>
        Understand AI in the physical world. Learn embodied intelligence, perception,
        decision making, and how AI interacts with physics-based robotic systems.
      </>
    ),
  },
  {
    title: "Humanoid Robot Middleware (ROS 2)",
    description: (
      <>
        Learn ROS 2 architecture, nodes, topics, and services to build a robotic
        nervous system capable of controlling humanoid motion in simulation and labs.
      </>
    ),
  },
  {
    title: "NVIDIA Isaac & VLA Capstone",
    description: (
      <>
        Work on vision-language-action (VLA), speech-to-action, path planning
        and object manipulation using high-fidelity robotics simulators + AI agents.
      </>
    ),
  },
  {
    title: "Real-World Autonomy",
    description: (
      <>
        Build autonomous humanoids that receive natural commands, plan multi-step
        tasks, avoid obstacles and manipulate objects using sensors like cameras,
        LiDAR, and IMUs.
      </>
    ),
  },
];

function Feature({ title, description }: FeatureItem) {
  return (
    <div className={clsx("col col--3", styles.neonCard, "group")}>
      <div className={styles.cardContent}>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
      <div className={styles.particles}></div> {/* Animated particles background */}
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.row}>
          {FeatureList.map((item, i) => (
            <Feature key={i} title={item.title} description={item.description} />
          ))}
        </div>
      </div>
    </section>
  );
}

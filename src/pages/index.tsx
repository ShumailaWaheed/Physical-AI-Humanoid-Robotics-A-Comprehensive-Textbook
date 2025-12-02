import type {ReactNode} from'react';
import clsx from'clsx';
import Link from'@docusaurus/Link';
import useDocusaurusContext from'@docusaurus/useDocusaurusContext';
import Layout from'@theme/Layout';
import HomepageFeatures from'@site/src/components/HomepageFeatures';
import Heading from'@theme/Heading';
import styles from'./index.module.css';

function HomepageHeader(){
const{siteConfig}=useDocusaurusContext();
return(
<header
className={clsx('hero hero--primary',styles.heroBanner)}
style={{
backgroundImage:"url('/img/hero-01.jpg')",
backgroundSize:'cover',
backgroundPosition:'center',
}}>

<img src='/img/robot.png'className={styles.heroImage}/>

<div className='dotWrap'>
<span className='dot'>✨</span>
<span className='dot'>✦</span>
<span className='dot'>✺</span>
<span className='dot'>✹</span>
<span className='dot'>✧</span>
<span className='dot'>✷</span>
</div>

<div className='container'>
<Heading as='h1'className='hero__title'>
Physical AI & Humanoid Robotics ⚡
</Heading>
<p className='hero__subtitle'>
Learn embodied intelligence, robot perception, motion planning
and AI-driven physical autonomy using ROS 2, Gazebo, Unity and NVIDIA Isaac.
</p>
<div className={styles.buttons}>
<Link className={clsx('button button--secondary button--lg',styles.hoverBtn)}to='/docs/intro'>
Start Learning 🚀
</Link>
</div>
</div>
</header>
)}
export default function Home():ReactNode{
const{siteConfig}=useDocusaurusContext();
return(
<Layout
title={`Welcome to ${siteConfig.title}`}
description='AI systems operating in the real physical world. Learn robotics + AI agents + humanoid autonomy.'>
<HomepageHeader/>
<main style={{background:'#01040f',paddingTop:'30px',}}>
<HomepageFeatures/>
</main>
</Layout>
)}

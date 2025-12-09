import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive guide to Physical AI and Humanoid Robotics for students and researchers',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // Site URL and base URL
  url: 'https://physical-ai-robotics.vercel.app',
  baseUrl: '/',

  organizationName: 'physical-ai-robotics',
  projectName: 'physical-ai-robotics',

  // Broken links configuration
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs',
          include: [
            'introduction/**/*.md',
            'module1-ros2/**/*.md',
            'module2-digital-twin/**/*.md',
            'module3-ai-robot-brain/**/*.md',
            'module4-vla-capstone/**/*.md',
          ],
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/your-username/physical-ai-robotics-book/edit/main/',
          routeBasePath: '/', // docs served from root
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/your-username/physical-ai-robotics-book/edit/main/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoids & Physical AI',
      logo: {
        alt: 'Humanoids & Physical AI Logo',
        src: 'img/logo.png',
        width: "50px",
        height: "60px"
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/ShumailaWaheed/Physical-AI-Humanoid-Robotics-A-Comprehensive-Textbook.git',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          items: [
            { label: 'Robotic Nervous System (ROS 2)', to: '/module1-ros2/ros2-fundamentals' },
            { label: 'Digital Twin (Gazebo & Unity)', to: '/module2-digital-twin/digital-twins-in-robotics' },
            { label: 'AI-Robot Brain (NVIDIA Isaac)', to: '/module3-ai-robot-brain/isaac-sim-overview' },
            { label: 'Vision-Language-Action (VLA + LLMs)', to: '/module4-vla-capstone/vla-fundamentals' },
          ],
        },
      ],
      copyright: 'Shumaila Waheed',
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

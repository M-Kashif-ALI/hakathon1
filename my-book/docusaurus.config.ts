import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'Educational module covering ROS 2 as middleware for humanoid robots',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://M-Kashif-ALI.github.io', // GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'M-Kashif-ALI', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics', // Usually your repo name.

  onBrokenLinks: 'throw',


  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
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
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    metadata: [
      {name: 'keywords', content: 'Physical AI, humanoid robotics, ROS 2, robotics, humanoid robots, middleware, rclpy, URDF, robot operating system, AI'},
      {name: 'description', content: 'Educational course covering Physical AI & Humanoid Robotics, including ROS 2 architecture, Python robot control, and URDF humanoid structure definition.'},
      {name: 'author', content: 'Physical AI & Humanoid Robotics Course Documentation Team'},
      {name: 'og:title', content: 'Physical AI & Humanoid Robotics Course - Educational Module'},
      {name: 'og:description', content: 'Comprehensive educational course covering Physical AI & Humanoid Robotics'},
      {name: 'og:type', content: 'website'},
      {name: 'og:url', content: 'https://M-Kashif-ALI.github.io/physical-ai-humanoid-robotics/'},
    ],
    navbar: {
      title: 'Physical AI & Humanoid Robotics Course',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          href: 'https://github.com/M-Kashif-ALI/hakathon1/tree/002-footer-docs-update',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'ROS 2 Robotic Nervous System',
              to: '/docs/ros2-nervous-system',
            },
            {
              label: 'Digital Twin',
              to: '/docs/digital-twin/gazebo-simulation/',
            },
            {
              label: 'Isaac Robot Brain',
              to: '/docs/isaac-robot-brain/isaac-sim/',
            },
            {
              label: 'VLA Integration',
              to: '/docs/vla-integration',
            },
          ],
        },
        {
          title: 'ROS Community',
          items: [
            {
              label: 'ROS Documentation',
              href: 'https://docs.ros.org/',
            },
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
            {
              label: 'ROS Discourse',
              href: 'https://discourse.ros.org/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/M-Kashif-ALI/hakathon1/tree/002-footer-docs-update',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

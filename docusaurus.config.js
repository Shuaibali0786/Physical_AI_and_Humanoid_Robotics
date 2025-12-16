// @ts-check
const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Colearning Agentic AI with ROS 2, Gazebo and NVIDIA Isaac',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'Shuaibali0786',
  projectName: 'ai-physical-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // 🌙☀️ Color Mode Toggle Configuration
      colorMode: {
        defaultMode: 'dark',              // Default dark mode (reading ke liye better)
        disableSwitch: false,             // Toggle button show hoga
        respectPrefersColorScheme: false, // User ki preference ignore
      },

      navbar: {
        title: 'AI Humanoid Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          // Sun/Moon toggle automatically right side pe aa jayega
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Shuaibali0786',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/yourprofile',
              },
            ],
          },
        ],
        copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
      },
    }),
};

module.exports = config;
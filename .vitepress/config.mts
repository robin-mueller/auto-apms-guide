import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  lang: 'en-US',
  title: 'AutoAPMS',
  description: 'Robotics Software Framework for Automated Action Planning and Mission Safeguarding (APMS) to enable fully autonomous operation. Works with ROS 2 and PX4 Autopilot.',
  base: '/auto-apms-guide/',

  head: [
    ['link', { rel: 'icon', type: 'image/x-icon', href: '/auto-apms-guide/favicon.ico' }],
    ['link', { rel: 'apple-touch-icon', sizes: '57x57', href: '/auto-apms-guide/apple-icon-57x57.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '60x60', href: '/auto-apms-guide/apple-icon-60x60.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '72x72', href: '/auto-apms-guide/apple-icon-72x72.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '76x76', href: '/auto-apms-guide/apple-icon-76x76.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '114x114', href: '/auto-apms-guide/apple-icon-114x114.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '120x120', href: '/auto-apms-guide/apple-icon-120x120.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '144x144', href: '/auto-apms-guide/apple-icon-144x144.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '152x152', href: '/auto-apms-guide/apple-icon-152x152.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '180x180', href: '/auto-apms-guide/apple-icon-180x180.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '192x192', href: '/auto-apms-guide/android-icon-192x192.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '32x32', href: '/auto-apms-guide/favicon-32x32.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '96x96', href: '/auto-apms-guide/favicon-96x96.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '16x16', href: '/auto-apms-guide/favicon-16x16.png' }],
    ['link', { rel: 'manifest', href: '/auto-apms-guide/manifest.json' }],
    ['meta', { name: 'msapplication-TileColor', content: '#74c7c7' }],
    ['meta', { name: 'msapplication-TileImage', content: '/auto-apms-guide/ms-icon-144x144.png' }],
    ['meta', { name: 'theme-color', content: '#74c7c7' }],
    ['meta', { property: 'og:type', content: 'website' }],
    ['meta', { property: 'og:locale', content: 'en' }],
    ['meta', { property: 'og:title', content: 'AutoAPMS - Smarter Missions with Safer Outcomes' }],
    ['meta', { property: 'og:site_name', content: 'AutoAPMS' }],
    ['meta', { property: 'og:image', content: 'https://robin-mueller.github.io/auto-apms-guide/favicon-96x96.png' }],
    ['meta', { property: 'og:url', content: 'https://robin-mueller.github.io/auto-apms-guide/' }],

    // Google Search Console
    ['meta', { name: 'google-site-verification', content: 'deYaqCwJq_6IydHhpWd_eiMjPjwJJ_yelf0aAgET3Ow' }]
  ],

  srcExclude: ['**/readme.md', '**/README.md', '**/TODO.md'],

  cleanUrls: true,

  sitemap: {
    hostname: 'https://robin-mueller.github.io/auto-apms-guide/'
  },

  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    siteTitle: 'AutoAPMS',
    externalLinkIcon: true,
    logo: '/favicon-96x96.png',
    nav: [
      { text: 'User Guide', link: '/introduction/overview' },
      { text: 'API Reference', link: 'https://robin-mueller.github.io/auto-apms' }
    ],

    sidebar: [
      {
        text: 'Introduction',
        items: [
          { text: 'Overview', link: '/introduction/overview' },
          { text: 'Getting started', 'link': '/introduction/getting-started'}
        ]
      },
    ],
    socialLinks: [
      { icon: 'github', link: 'https://github.com/robin-mueller/auto-apms' }
    ],
    editLink: {
      pattern: 'https://github.com/robin-mueller/auto-apms-guide/blob/master/:path'
    },
    lastUpdated: {
      text: 'Last update',
      formatOptions: {
        dateStyle: 'medium',
        timeStyle: 'short'
      }
    },
    footer: {
      message: 'Released under the Apache-2.0 License.',
      copyright: 'Copyright © 2024-present Robin Müller'
    }
  }
})

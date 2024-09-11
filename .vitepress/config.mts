import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  lang: 'en-US',
  title: 'AutoAPMS',
  description: 'Smarter Missions with Safer Outcomes',
  base: '/auto-apms/',

  head: [
    // Favicon
    ['link', { rel: 'icon', href: '/auto-apms/favicon.ico' }],
    ['link', { rel: 'apple-touch-icon', sizes: '57x57', href: '/auto-apms/apple-icon-57x57.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '60x60', href: '/auto-apms/apple-icon-60x60.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '72x72', href: '/auto-apms/apple-icon-72x72.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '76x76', href: '/auto-apms/apple-icon-76x76.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '114x114', href: '/auto-apms/apple-icon-114x114.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '120x120', href: '/auto-apms/apple-icon-120x120.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '144x144', href: '/auto-apms/apple-icon-144x144.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '152x152', href: '/auto-apms/apple-icon-152x152.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '180x180', href: '/auto-apms/apple-icon-180x180.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '192x192', href: '/auto-apms/android-icon-192x192.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '32x32', href: '/auto-apms/favicon-32x32.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '96x96', href: '/auto-apms/favicon-96x96.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '16x16', href: '/auto-apms/favicon-16x16.png' }],
    ['link', { rel: 'manifest', href: '/auto-apms/manifest.json' }],
    ['meta', { name: 'msapplication-TileColor', content: '#ffffff' }],
    ['meta', { name: 'msapplication-TileImage', content: '/auto-apms/ms-icon-144x144.png' }],
    ['meta', { name: 'theme-color', content: '#ffffff' }],

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
      { text: 'User Guide', link: '/welcome/' },
      { text: 'API Reference', link: 'https://robin-mueller.github.io/auto-apms' }
    ],

    sidebar: [
      {
        text: 'Introduction',
        items: [
          { text: 'Overview', link: '/welcome/' }
        ]
      },
      // {
      //   text: 'Examples',
      //   items: [
      //     { text: 'Markdown Examples', link: '/markdown-examples' },
      //     { text: 'Runtime API Examples', link: '/api-examples' }
      //   ]
      // }
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

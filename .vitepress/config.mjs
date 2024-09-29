import { defineConfig } from 'vitepress'
import { generateSidebar } from 'vitepress-sidebar'

const vitepressSidebarOptions = {
  documentRootPath: 'src',
  useFolderLinkFromIndexFile: true,
  useFolderTitleFromIndexFile: true,
  useTitleFromFileHeading: true,
  useTitleFromFrontmatter: true,
  collapseDepth: 2,
  sortMenusByFrontmatterOrder: true
}

// https://vitepress.dev/reference/site-config
export default defineConfig({
  lang: 'en-US',
  title: 'AutoAPMS',
  description: 'A Software Framework for Automated (Auto) Action Planning and Mission Safeguarding (APMS) in Robotics. Works with ROS 2 and PX4 Autopilot.',
  base: '/auto-apms-guide/',

  head: [
    ['link', { rel: 'icon', type: 'image/x-icon', href: '/auto-apms-guide/logo/favicon.ico' }],
    ['link', { rel: 'apple-touch-icon', sizes: '57x57', href: '/auto-apms-guide/logo/apple-icon-57x57.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '60x60', href: '/auto-apms-guide/logo/apple-icon-60x60.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '72x72', href: '/auto-apms-guide/logo/apple-icon-72x72.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '76x76', href: '/auto-apms-guide/logo/apple-icon-76x76.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '114x114', href: '/auto-apms-guide/logo/apple-icon-114x114.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '120x120', href: '/auto-apms-guide/logo/apple-icon-120x120.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '144x144', href: '/auto-apms-guide/logo/apple-icon-144x144.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '152x152', href: '/auto-apms-guide/logo/apple-icon-152x152.png' }],
    ['link', { rel: 'apple-touch-icon', sizes: '180x180', href: '/auto-apms-guide/logo/apple-icon-180x180.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '192x192', href: '/auto-apms-guide/logo/android-icon-192x192.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '32x32', href: '/auto-apms-guide/logo/favicon-32x32.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '96x96', href: '/auto-apms-guide/logo/favicon-96x96.png' }],
    ['link', { rel: 'icon', type: 'image/png', sizes: '16x16', href: '/auto-apms-guide/logo/favicon-16x16.png' }],
    ['link', { rel: 'manifest', href: '/auto-apms-guide/logo/manifest.json' }],
    ['meta', { name: 'msapplication-TileColor', content: '#74c7c7' }],
    ['meta', { name: 'msapplication-TileImage', content: '/auto-apms-guide/logo/ms-icon-144x144.png' }],
    ['meta', { name: 'theme-color', content: '#74c7c7' }],
    ['meta', { property: 'og:url', content: 'https://robin-mueller.github.io/auto-apms-guide/' }],
    ['meta', { property: 'og:type', content: 'website' }],
    ['meta', { property: 'og:locale', content: 'en' }],
    ['meta', { property: 'og:title', content: 'AutoAPMS - Smarter Missions with Safer Outcomes' }],
    ['meta', { property: 'og:site_name', content: 'AutoAPMS' }],
    ['meta', { property: 'og:image', content: 'https://robin-mueller.github.io/auto-apms-guide/logo/logo-og.png' }],
    ['meta', { property: 'og:image:width', content: '1306' }],
    ['meta', { property: 'og:image:height', content: '910' }],
    ['meta', { name: 'twitter:card', content: 'summary_large_image' }],
    ['meta', { property: 'twitter:domain', content: 'robin-mueller.github.io' }],
    ['meta', { property: 'twitter:url', content: 'https://robin-mueller.github.io/auto-apms-guide/' }],
    ['meta', { name: 'twitter:title', content: 'AutoAPMS - Smarter Missions with Safer Outcomes' }],
    ['meta', { name: 'twitter:image', content: 'https://robin-mueller.github.io/auto-apms-guide/logo/logo-og.png' }],

    // Google Search Console
    ['meta', { name: 'google-site-verification', content: 'deYaqCwJq_6IydHhpWd_eiMjPjwJJ_yelf0aAgET3Ow' }]
  ],

  srcDir: 'src',
  srcExclude: ['**/readme.md', '**/README.md', '**/TODO.md'],

  cleanUrls: true,

  sitemap: {
    hostname: 'https://robin-mueller.github.io/auto-apms-guide/'
  },

  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    siteTitle: 'AutoAPMS',
    externalLinkIcon: true,
    logo: '/logo/favicon-96x96.png',

    search: {
      provider: "local",
    },

    nav: [
      { text: 'User Guide', link: '/intro', activeMatch: '/\\S' },
      { text: 'API Reference', link: 'https://robin-mueller.github.io/auto-apms' }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/robin-mueller/auto-apms' }
    ],

    sidebar: generateSidebar(vitepressSidebarOptions),

    editLink: {
      pattern: 'https://github.com/robin-mueller/auto-apms-guide/blob/master/src/:path'
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

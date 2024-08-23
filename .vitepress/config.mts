import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  lang: 'en-US',
  title: 'PX4 Behavior',
  description: 'Flexible behavior-based task management for automated systems integrating with PX4.',
  base: '/px4-behavior-docs/',
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    siteTitle: false,
    logo: '/logo.png',
    nav: [
      { text: 'API Reference', link: 'https://robin-mueller.github.io/px4-behavior' },
    ],
    
    sidebar: [
      {
        text: 'Examples',
        items: [
          { text: 'Markdown Examples', link: '/markdown-examples' },
          { text: 'Runtime API Examples', link: '/api-examples' }
        ]
      }
    ],
    socialLinks: [
      { icon: 'github', link: 'https://github.com/robin-mueller/px4-behavior' }
    ],
    editLink: {
      pattern: 'https://github.com/robin-mueller/px4-behavior-docs/blob/master/:path'
    },
    lastUpdated: {
      text: 'Last update',
      formatOptions: {
        dateStyle: 'medium',
        timeStyle: 'short'
      }
    },
    externalLinkIcon: true
  }
})

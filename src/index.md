---
# https://vitepress.dev/reference/default-theme-home-page
layout: home
titleTemplate: :title - Resilient Robot Mission Management

hero:
  name: AutoAPMS
  text: Resilient Robot Mission Management
  tagline: Smarter Missions with Safer Outcomes
  image:
    src: /logo/mainpage-icon.png
    width: 250
  actions:
    - theme: brand
      text: User Guide
      link: /intro
    - theme: alt
      text: API Reference
      link: https://robin-mueller.github.io/auto-apms
    - theme: alt
      text: GitHub
      link: https://github.com/robin-mueller/auto-apms

features:
  - title: Compatible with PX4 Autopilot
    icon: 
      src: /drone-light.svg
    details: This software framework has been designed for safeguarding missions executed by unmanned systems running PX4
  - title: Leveraging modern ROS 2
    icon: 
      src: /cpu-light.svg
    details: Real-time processes are managed by the state-of-the-art ROS 2 middleware
  - title: Comprehensible Task Planning
    icon: 
      src: /routing-light.svg
    details: Thanks to the behavior-based software design, actions or tasks can be created using a high level of abstraction
  - title: Extensible and Modular
    icon: 
      src: /code-light.svg
    details: Develop your own tasks and control flow to create any kind of behavior you'd like
---

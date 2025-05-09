# AutoAPMS User Guide

[![Deploy](https://github.com/robin-mueller/auto-apms-guide/actions/workflows/deploy.yml/badge.svg?branch=master)](https://github.com/robin-mueller/auto-apms-guide/deployments)

The docs are powered by [VitePress](https://vitepress.dev/). The website is deployed by a GitHub action on every push to the `master` branch.

## Setup

Install [Node.js](https://nodejs.org/en). We recommend the latest LTS version (as for now it's 22). Here is how to do it on Ubuntu:

```sh
# Setup the local Ubuntu repository
curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E sh -

# Install Node.js
sudo apt-get install nodejs npm -y

# Install dependencies specified in package.json (Run in auto-apms-guide directory)
npm install
```

For further information, visit the [quickstart guide](https://vitepress.dev/guide/getting-started) for setting up VitePress.

## Build

The project is scaffolded from the root of this repository and supports the following commands:

- Run `npm run dev` for [starting the dev server](https://vitepress.dev/reference/cli#vitepress-dev)
- Run `npm run build` for [generating the static HTML site](https://vitepress.dev/reference/cli#vitepress-build)
- Run `npm run preview` to [preview the production build](https://vitepress.dev/reference/cli#vitepress-preview)
- Run `npm run lint` to lint the markdown
- Run `npm run lint-fix` to lint AND fix the markdown formatting

## Scripts

- [create_full_tree_node_reference.sh](./script/create_full_tree_node_reference.sh)

    Generate the markdown file that is used to display the behavior tree node reference page. This script requires a valid AutoAPMS workspace for parsing the builtin nodes.

    ```bash
    # Must be executed from repository root
    ./script/create_full_tree_node_reference.sh /path/to/auto-apms/workspace
    ```

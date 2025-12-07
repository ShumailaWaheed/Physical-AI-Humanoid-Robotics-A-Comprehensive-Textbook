# Quickstart

This document provides instructions on how to set up the development environment and build the Docusaurus-based book.

## Prerequisites

- **Node.js**: Version 18.x or later.
- **npm** or **yarn**: A Node.js package manager.

## Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```
    or
    ```bash
    yarn install
    ```

## Running the Development Server

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

```bash
npm run start
```
or
```bash
yarn start
```

The book will be available at `http://localhost:3000`.

## Building the Book

This command generates static content into the `build` directory and can be served using any static contents hosting service.

```bash
npm run build
```
or
```bash
yarn build
```

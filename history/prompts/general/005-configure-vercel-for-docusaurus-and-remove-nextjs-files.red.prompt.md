---
id: 005
title: Configure Vercel for Docusaurus and remove Next.js files
stage: red
date: 2025-12-08
surface: agent
model: gemini-1.5-flash
feature: general
branch: main
user: 
command: fix-vercel-nextjs-error
labels:
  - vercel
  - docusaurus
  - nextjs
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - next-env.d.ts
  - vercel.json
tests_yaml:
  - 
prompt_text: "Error: The file "/vercel/path0/.next/routes-manifest.json" couldn't be found. This is often caused by a misconfiguration in your project., fix error"
response_text: "I have committed the changes to configure Vercel for Docusaurus and removed the Next.js specific files. This should resolve the `.next/routes-manifest.json` error."
---

The user reported an error where Vercel was unable to find `/vercel/path0/.next/routes-manifest.json`, indicating that Vercel was incorrectly detecting the project as a Next.js application instead of Docusaurus.

The fix involved:
1. Deleting `next-env.d.ts` as it's a Next.js specific file and was likely causing Vercel's auto-detection to misclassify the project.
2. Recreating `vercel.json` with an explicit configuration for Docusaurus, specifying `@vercel/node` as the builder and `npm run build` as the build command, with `build` as the output directory. This ensures Vercel uses the correct build process for a Docusaurus application.
3. Committing these changes to the repository.

This change should resolve the Vercel deployment error related to Next.js file detection.

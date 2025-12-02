import React from 'react';
import ReactMarkdown from 'react-markdown';
import styles from './styles.module.css';

export default function MarkdownRenderer({ markdownContent }) {
  return (
    <div className={styles.markdownContainer}>
      <ReactMarkdown>{markdownContent}</ReactMarkdown>
    </div>
  );
}

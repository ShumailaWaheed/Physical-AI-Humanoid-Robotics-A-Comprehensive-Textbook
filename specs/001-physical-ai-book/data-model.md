# Data Model

This document outlines the key data entities for the "Physical AI & Humanoid Robotics Book" feature. As this feature is focused on generating a book (documentation), the data model describes the structure of the content itself, not a software application.

## Entity: Book

Represents the entire digital manuscript.

- **Attributes**:
  - `title` (string): The main title of the book.
  - `introduction` (string): The introductory text.
  - `modules` (list of Module): An ordered list of the main modules.
  - `word_count` (integer): The total word count of the book.

- **Validation Rules**:
  - `word_count` must be between 20,000 and 30,000.
  - Must contain exactly 4 `Module` entities.

## Entity: Module

Represents a major thematic section of the book.

- **Attributes**:
  - `title` (string): The title of the module (e.g., "The Robotic Nervous System (ROS 2)").
  - `learning_outcomes` (list of string): A list of what the reader will be able to do after completing the module.
  - `chapters` (list of Chapter): An ordered list of chapters within the module.

- **Relationships**:
  - Belongs to one `Book`.
  - Contains many `Chapters`.

## Entity: Chapter

Represents a specific, numbered section within a module.

- **Attributes**:
  - `title` (string): The title of the chapter (e.g., "ROS 2 Fundamentals").
  - `content` (string, Markdown): The main body of the chapter.
  - `exercises` (list of Practical Exercise): A list of exercises for the chapter.

- **Relationships**:
  - Belongs to one `Module`.

## Entity: Practical Exercise

Represents a hands-on task for the reader.

- **Attributes**:
  - `description` (string): A description of the exercise's goal.
  - `steps` (list of string): A sequence of steps to complete the exercise.
  - `code_snippets` (list of Code Snippet): Associated code for the exercise.

- **Validation Rules**:
  - Must be solvable by a user meeting the stated prerequisites.

## Entity: Code Snippet

Represents a small, illustrative piece of code.

- **Attributes**:
  - `language` (string): The programming language (e.g., "Python", "URDF").
  - `code` (string): The block of code itself.

## Entity: Diagram

Represents a visual illustration.

- **Attributes**:
  - `title` (string): The title/caption for the diagram.
  - `type` (enum): The type of diagram (e.g., "Workflow", "Architecture").
  - `source` (string): Path or reference to the image file.

## Entity: Citation

Represents a formal reference to an external source.

- **Attributes**:
  - `style` (enum): The citation style (e.g., "APA").
  - `text` (string): The formatted citation text.

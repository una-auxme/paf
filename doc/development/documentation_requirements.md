# Documentation Requirements

**Summary:** This document outlines the guidelines for documentation to ensure consistency, readability, maintainability, and adherence to best practices in coding standards.

- [1. Linting](#1-linting)
  - [1.1. Python Linting](#11-python-linting)
  - [1.2. Markdown Linting](#12-markdown-linting)
- [2. Templates](#2-templates)
- [3. Python Docstrings](#3-python-docstrings)
- [4. Readability and Maintainability](#4-readability-and-maintainability)
- [5. Code Structure](#5-code-structure)
- [6. Efficiency and Performance](#6-efficiency-and-performance)
- [7. Error Handling](#7-error-handling)
- [8. Testing](#8-testing)
- [9. Security](#9-security)
- [10. Documentation](#10-documentation)
- [11. Version Control](#11-version-control)
- [12. Scalability](#12-scalability)
- [13. Consistency with Coding Standards](#13-consistency-with-coding-standards)

## 1. Linting

You can lint your files locally with:

```bash
docker compose -f build/docker-compose.linting.yaml up
```

### 1.1. Python Linting

Ensure that all Python files pass linting checks without issues.
Tools such as `black` are already present in this project.
Regular linting ensures that the code adheres to the project's style guide, enhancing readability and maintainability.

### 1.2. Markdown Linting

Lint all markdown files with `markdownlint`.

## 2. Templates

Follow the guidelines presented in the following templates:

- [python](templates/template_class.py)
- [markdown](templates/template_wiki_page.md)

## 3. Python Docstrings

The project comes with an extension to automatically generate docstrings. It is mandatory to use it if you comment your code.

To generate a docstring you have to be in the development container and then a popup will apear after writing three quotation marks:

![docstring_popup.png](/doc/assets/development/docstring_popup.png)

Press `Enter` or select the option and it should produce a docstring that looks like this:

![docstring.png](/doc/assets/development/docstring.png)

## 4. Readability and Maintainability

- **Consistent Formatting:** Code should follow a consistent and readable formatting style. Tools like linters or formatters can help enforce a consistent code style.
  - [linting](./linting.md)
- **Meaningful Names:** Variable and function names should be descriptive and convey the purpose of the code.
- **Comments:** Clear and concise comments should be used where necessary to explain complex logic or provide context.

## 5. Code Structure

- **Modularity:** Code should be organized into modular components or functions, promoting reusability and maintainability.
- **Appropriate Use of Functions/Methods:** Functions should have a clear purpose and adhere to the single responsibility principle.
- **Hierarchy and Nesting:** Avoid overly nested structures; use appropriate levels of indentation to enhance readability.

## 6. Efficiency and Performance

- **Optimized Algorithms:** Code should use efficient algorithms and data structures to achieve good performance.
- **Avoidance of Code Smells:** Detect and eliminate code smells such as duplicated code, unnecessary complexity, or anti-patterns.

## 7. Error Handling

- **Effective Error Messages:** Error messages should be clear and provide useful information for debugging.
- **Graceful Error Handling:** Code should handle errors gracefully, avoiding crashes and providing appropriate feedback.

## 8. Testing

- **Comprehensive Test Coverage:** Code should be accompanied by a suite of tests that cover different scenarios, ensuring reliability and maintainability.
- **Test Readability:** Tests should be clear and easy to understand, serving as documentation for the codebase.

## 9. Security

- **Input Validation:** Code should validate and sanitize inputs.

## 10. Documentation

- **Code Comments:** In addition to in-code comments, consider external documentation for the overall project structure, APIs, and configurations.
- **README Files:** Include a well-written README file that provides an overview of the project, installation instructions, and usage examples.

## 11. Version Control

- **Commit Messages:** Use descriptive and meaningful commit messages to track changes effectively.
- **Branching Strategy:** Follow a consistent and well-defined branching strategy to manage code changes.

## 12. Scalability

- **Avoid Hardcoding:** Parameterize values that might change, making it easier to scale the application.
- **Optimized Resource Usage:** Ensure efficient utilization of resources to support scalability.

## 13. Consistency with Coding Standards

- **Adherence to Coding Guidelines:** Follow established coding standards and best practices for the programming language or framework used.

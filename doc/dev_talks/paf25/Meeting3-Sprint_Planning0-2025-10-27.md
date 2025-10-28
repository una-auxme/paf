# First Sprint Planning Meeting - October 27, 2025

**Date:** October 27, 2025
**Duration:** 14:00 - 16:30
**Topic:** Sprint Planning & Project Introduction

---

## Agenda

### 14:00 - Preparation

- **Collect GitHub usernames** from all participants
- Ensure everyone has repository access

### 14:00 - 14:30 - Tool Introduction

#### Introduction to Debugging

Learn how to effectively debug the PAF agent using:

- VS Code's built-in debugger
- Attaching to running ROS 2 nodes
- Setting breakpoints in Python nodes
- Inspecting variables and call stacks

**Documentation:** [Debugging Guide](../general/debugging.md)

#### Introduction to PyTrees Visualization

Understanding behavior trees with visualization tools:

- `py_trees_ros_viewer` for live behavior tree monitoring
- Understanding the behavior tree structure
- Debugging decision-making logic

---

### 14:30 - 15:15 - Brainstorming Session

#### Overview of Current Challenges

- Identify the **biggest pain points** in the current system
- Discuss areas needing improvement or refactoring
- Review known issues and technical debt

**Reference:** [Architecture Overview](../../acting/architecture_documentation.md)

#### Create Issues from Discussion

- Transform identified problems into **actionable GitHub issues**
- Use clear, descriptive titles and detailed descriptions
- Add appropriate labels and priority tags

---

### 15:15 - 15:45 - Git Workflow & Project Management

#### Issue Management

- **Check for duplicate issues** before creating new ones
- How to properly **create and structure issues**:
  - Clear title and description
  - Add appropriate **tags/labels** (bug, enhancement, documentation, etc.)
  - Assign to the right **project board**
  - Set priorities and milestones

#### Pull Request Process

- **How to create a Pull Request (PR)**
- Link PRs to related issues
- Use the **Sprint Summary template** for documentation
- Follow **Sprint Review Presentation guidelines**

**Documentation:**

- [Git Workflow](../../development/git_workflow.md)
- [Review Guidelines](../../development/review_guideline.md)
- [Sprint Review Presentation](../../development/sprint_review_presentation.md)
- [Sprint Planning Guidelines](../../development/sprint_planning_guidelines.md)

#### PR Review Process

- How to **review pull requests effectively**
- What to look for in code reviews
- Providing constructive feedback
- Approval process and merging

**Note:** Regular meeting schedule will be established starting next meeting.

---

### 15:45 - 16:15 - Topic Distribution & Issue Creation

#### Distribute Topic Areas

- **Assign team members** to different system components:
  - Perception
  - Localization
  - Mapping
  - Planning
  - Control
- Consider experience levels and interests

#### Independent Issue Work

- Team members **create or update issues** for their assigned areas
- Document technical requirements
- Propose solutions or approaches
- Add technical details and acceptance criteria

---

### 16:15 - 16:30 - Issue Review & Prioritization

#### Review Created Issues

- Go through all newly created/updated issues as a team
- **Assign issues** to specific team members
- **Set priorities** (High, Medium, Low)
- Plan what will be tackled in the upcoming sprint

#### Sprint Planning

- Define sprint goals
- Estimate effort for issues
- Commit to deliverables

---

## Important Guidelines

### Research Documentation

All research results must be documented as **Markdown files** in the repository:

- Place research documents in the appropriate `doc/research/` subfolder
- Follow the documentation structure of previous semesters
- Include references, diagrams, and code examples where applicable

**Documentation:** [Documentation Requirements](../../development/documentation_requirements.md)

### Project Management

- Use GitHub Projects for tracking sprint progress
- Update issue status regularly
- Link commits and PRs to issues using keywords (e.g., `Fixes #123`)

**Documentation:** [Project Management](../../development/project_management.md)

---

## Preparation for Next Meeting

Before the next meeting, please:

1. ✅ Complete assigned issues or provide status updates
2. ✅ Review open PRs in your area
3. ✅ Document any blockers or challenges
4. ✅ Prepare questions for the team

---

## Resources

### Development Resources

- [First Steps Guide](../development/first_steps.md)
- [Execution Guide](../general/execution.md)
- [Testing Guide](../general/tests.md)
- [Installation Guide](../general/installation.md)

### Project Overview

- [Current Architecture](../general/architecture_current.md)
- [README](../general/README.md)

### Development Guidelines

- [Git Workflow](../../development/git_workflow.md)
- [Review Guidelines](../../development/review_guideline.md)
- [Linting Guide](../../development/linting.md)

---

## Questions?

If you have any questions or need clarification:

- Post in the team communication channel
- Check the documentation links above
- Ask during the next meeting
- Contact the maintainers: @JulianTrommer and @ll7

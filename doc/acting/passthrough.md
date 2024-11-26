# Passthrough

>[!NOTE]
>The passthrough component is a temporary solution while beeing in the transitioning phase of
>splitting up the acting component into acting and control.

**Summary:** This page is ought to provide an overview of the passthrough node and
reason why it exists

- [Overview](#overview)
- [Reasoning](#reasoning)

## Overview

The passthrough node subscribes to topics on a global scale and republishes them into the "paf/acting" namespace. The control package subscribes to these re-emitted topics.

> [!NOTE]
> See [acting architecture](./architecture_documentation.md) for further information.

## Reasoning

Before the control package was outsourced and became its own package it resided within the acting package.
Therefor many global dependencies existed in the control package. As the goal of the outsourcing was
to eliminate global dependencies in control theory the passthrough node was created as a first stepping
stone towards independence.

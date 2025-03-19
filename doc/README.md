# PAF Documentation

This document provides an overview of the structure of the documentation.

- [`general`](#general)
- [`development`](#development)
- [`research`](#research)
- [`examples`](#examples)
- [`perception`](#perception)
- [`mapping`](#mapping)
- [`planning`](#planning)
- [`acting`](#acting)
- [`assets`](#assets)
- [`dev_talks`](#dev_talks)

## `general`

The [`general`](./general/) folder contains installation instructions for the project and an overview of the system architecture.

## `development`

The [`development`](./development/) folder contains guidelines for developing inside the project. It also provides templates for documentation files and python classes. Further information can be found in the [README](development/README.md).

## `research`

The [`research`](./research/) folder contains the findings of each group during the initial phase of the project.

## `examples`

To-do

## `perception`

The [`perception`](./perception/) folder contains documentation for the whole perception module and its individual components.

## `mapping`

The [`mapping`](./mapping/) folder contains documentation for the mapping package, often also called the **Intermediate Layer**.

The **Intermediate Layer** receives most sensor information (everything except traffic light) from [perception](#perception), puts it into a
unified data format: [Map](./mapping/generated/mapping_common/map.md#map) and then forwards it to [planning](#planning)/[acting](#acting)

## `planning`

The [`planning`](./planning/) folder contains documentation for the whole planning module and its individual components.

## `acting`

The [`acting`](./acting/) folder contains documentation for the whole acting module and its individual components.

## `assets`

The [`assets`](./assets/) folder contains mainly images that are used inside the documentation.

## `dev_talks`

The [`dev_talks`](./dev_talks/README.md) folder contains the protocols of each sprint review and roles that the students fill during the project.

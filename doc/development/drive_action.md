# GitHub actions

**Summary:** This page explains the GitHub build action we use to evaluate our agent.

- [The drive job](#the-drive-job)
  - [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository-actionscheckoutv3)
  - [2. Download artifact](#2-download-artifact)
  - [3. Unzip artifact](#3-unzip-artifact)
  - [4. Return artifact JSON](#4-return-artifact-json)
  - [5. Run agent with docker-compose](#5-run-agent-with-docker-compose)
  - [6. Copy simulation results file out of container](#6-copy-simulation-results-file-out-of-container)
  - [7. Stop docker-compose stack](#7-stop-docker-compose-stack)
  - [8. Create simulation results table](#8-create-simulation-results-table)
  - [9. Print simulation results](#9-print-simulation-results)
  - [10. Comment result in pull request `actions/github-script@v6`](#10-comment-result-in-pull-request-actionsgithub-scriptv6)
- [Simulation results](#simulation-results)
  - [11. Prune all images older than one day](#11-prune-all-images-older-than-one-day)

## The drive job

The `drive` job is executed conditionally on `pull_request`, after the build successfully ran through.

> Warning: Always start the GitHub runner that handles the `drive` action through direct access to the machine. Do not use remote access like `ssh` or `xrdp`.

### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Same step as in the [build job](#1-checkout-repository--actionscheckoutv3-)

### 2. Download artifact

Downloads the artifact that was uploaded during the preceeding `build` action.

### 3. Unzip artifact

Extracts the files of the downloaded artifact.

### 4. Return artifact JSON

Parses the extracted file in the JSON format to read the information inside the file.

### 5. Run agent with docker-compose

Runs the agent with the [`build/docker-compose.cicd.yaml`](../../build/docker-compose.cicd.yaml) that only contains the
bare minimum components for test execution:

- Carla Simulator
- roscore
- Agent container, run through the
  Carla [`leaderboard_evaluator`](https://github.com/carla-simulator/leaderboard/blob/leaderboard-2.0/leaderboard/leaderboard_evaluator.py).

### 6. Copy simulation results file out of container

Copies the created `simulation_results.json` file out of the agent container into the current container

### 7. Stop docker-compose stack

Stops the remaining containers (Carla, roscore) and removes the volumes with:
`$ docker-compose down -v`.

This step is important to clean up the remaining containers to have a clean run everytime. This is also the reason for
the `if: always()`, that ensures step execution.

### 8. Create simulation results table

Reads the simulation results an creates a table for better readability.

### 9. Print simulation results

Prints the simulation results table to the action.

### 10. Comment result in pull request [`actions/github-script@v6`](https://github.com/marketplace/actions/github-script)

This steps uses a JS script to parse the simulation results and add a comment with a results table to the corresponding
pull request.

An example comment for this would be:

## Simulation results

| Metric                               | Value   |
|--------------------------------------|---------|
| Avg. driving score                   | 0.06006 |
| Avg. route completion                | 0.22    |
| Avg. infraction penalty              | 0.273   |
| Collisions with pedestrians          | 0.0     |
| Collisions with vehicles             | 62.046  |
| Collisions with layout               | 62.046  |
| Red lights infractions               | 0.0     |
| Stop sign infractions                | 0.0     |
| Off-road infractions                 | 0       |
| Route deviations                     | 0.0     |
| Route timeouts                       | 62.046  |
| Agent blocked                        | 0.0     |
| Yield emergency vehicles infractions | 0.0     |
| Scenario timeouts                    | 62.046  |
| Min speed infractions                | 0.0     |

### 11. Prune all images older than one day

Removes all images from the runner that are older than one day to free disk space.

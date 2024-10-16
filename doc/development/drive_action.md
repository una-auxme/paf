# GitHub actions

**Summary:** This page explains the GitHub build action we use to evaluate our agent.

- [GitHub actions](#github-actions)
  - [The drive job](#the-drive-job)
    - [1. Checkout repository (`actions/checkout@v3`)](#1-checkout-repository-actionscheckoutv3)
    - [2. Run agent with docker-compose](#2-run-agent-with-docker-compose)
    - [3. Copy simulation results file out of container](#3-copy-simulation-results-file-out-of-container)
    - [4. Stop docker-compose stack](#4-stop-docker-compose-stack)
    - [5. Comment result in pull request `actions/github-script@v6`](#5-comment-result-in-pull-request-actionsgithub-scriptv6)
  - [Simulation results](#simulation-results)

## The drive job

The `drive` job is executed conditionally on `pull_request`, after the build successfully ran through.

### 1. Checkout repository ([`actions/checkout@v3`](https://github.com/actions/checkout))

Same step as in the [build job](#1-checkout-repository--actionscheckoutv3-)

### 2. Run agent with docker-compose

Runs the agent with the [`build/docker-compose.cicd.yaml`](../../build/docker-compose.cicd.yaml) that only contains the
bare minimum components for test execution:

- Carla Simulator
- roscore
- Agent container, run through the
  Carla [`leaderboard_evaluator`](https://github.com/carla-simulator/leaderboard/blob/leaderboard-2.0/leaderboard/leaderboard_evaluator.py).

### 3. Copy simulation results file out of container

Copies the created `simulation_results.json` file out of the agent container into the current container

### 4. Stop docker-compose stack

Stops the remaining containers (Carla, roscore) and removes the volumes with:
`$ docker-compose down -v`.

This step is important to clean up the remaining containers to have a clean run everytime. This is also the reason for
the `if: always()`, that ensures step execution.

### 5. Comment result in pull request [`actions/github-script@v6`](https://github.com/marketplace/actions/github-script)

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

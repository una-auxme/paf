# How to load checkpoints

**Summary:** This page provides instructions on how to load specific scenarios in the Carla simulator to avoid having to wait for a specific event to occur.

## Instruction

1. Go to the agent_service.yaml file in the build directory.
2. Comment out the standard leaderboard route if it is in. ('''- ROUTE=/opt/leaderboard/data/routes_devtest.xml''' ln:32)
3. Comment in your specific scenario you want. The specific scenarios start at line 34.
4. Now Compose up the docker-compose.leaderboard.yaml file.

## Scenario Map

Will be added soon

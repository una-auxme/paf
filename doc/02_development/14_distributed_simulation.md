# Distributed Simulation

If you have not enough compute resources, start the `carla-simulator-server' on a remote machine and execute the agent on your local machine.

## Author

Julian Trommer and Lennart Luttkus

## Date

2024-06-28

## Remote Machine Setup

- Gain `ssh` access to the remote machine.
- Download the [Carla leaderboard release](https://leaderboard.carla.org/get_started/#11-download-the-carla-leaderboard-package) to the remote PC
- Extract the `.zip` file.
- start the server per ssh without rendering the spectator view:
  - `/CARLA_Leaderboard_20/CarlaUE4.sh -RenderOffScreen`

## Local Machine Setup

- get access to the remote machine via `ssh`
  - Start the server as described above
- set the host ip address from the remote machine as the new carla-ip address
- start the agent on your local machine
  
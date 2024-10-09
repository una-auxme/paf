# Distributed Simulation

If you have not enough compute resources, start the `carla-simulator-server` on a remote machine and execute the agent on your local machine.
As far as we know, you need more than **10 GB of VRAM** to run the server and the agent on the same machine.

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

### Ensure similarity between normal docker-compose and distributed docker-compose files

Carefully compare that their are no major differences between the `docker-compose.yml` and `docker-compose.distributed.yml` files.
Mainly, the `carla-simulator` service will not be executed in the non-distributed version.

### Set the `<ip-address>` of the carla simulator in docker-compose distributed files

Replace the argument `<carla-server-ip-address>` with the ip address of the remote machine.
You can find the ip address of the remote machine by executing the following command on the remote machine:

```bash
hostname -I
```

Typically, the ip address is the first one in the list.
`172.xxx.xxx.xxx` is the localhost address and not the relevant address.

Replace the ip-address in the following files:

- `build/docker-compose.devroute-distributed.yaml`
- `build/docker-compose.leaderboard-distributed.yaml`

### Start the agent on your local machine

Navigate to the files mentioned above in the VS Code Explorer and select `Compose Up` after right-clicking one of the files.

## How do you know that you do not have enough compute resources?

```bash
watch -n 1 nvidia-smi
```

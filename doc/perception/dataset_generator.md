# Dataset generator

**Summary:** The dataset generator located in perception/src/dataset_generator.py is a standalone script, directly
hooking into the Carla Python API. It is used to generate a dataset to train perception models.

- [Dataset generator](#dataset-generator)
  - [Necessary adjustments](#necessary-adjustments)
    - [Dockerfile](#dockerfile)
    - [docker-compose.yml](#docker-composeyml)
  - [Usage](#usage)
    - [Using with leaderboard](#using-with-leaderboard)

## Necessary adjustments

Important to note: The dataset generator uses
the [instance segmentation camera](https://carla.readthedocs.io/en/0.9.14/ref_sensors/#instance-segmentation-camera),
which is only available in Carla 0.9.14 and above.
Unfortunately, the leaderboard 2.0 is still based on Carla 0.9.13+, so to use the dataset generator,
the agent [Dockerfile](../../build/docker/agent/Dockerfile), as well as
the [docker-compose.yml](../../build/docker-compose.yml)
need to be adjusted:

### Dockerfile

```diff
diff --git a/build/docker/agent/Dockerfile b/build/docker/agent/Dockerfile
index 032189d..d08f24a 100644
--- a/build/docker/agent/Dockerfile
+++ b/build/docker/agent/Dockerfile
@@ -2,8 +2,8 @@
 # Based on https://github.com/ll7/paf21-1/blob/master/components/carla_ros_bridge/Dockerfile
 
 # Use this image to enable instance segmentation cameras
-# FROM carlasim/carla:0.9.14 as carla
-FROM ghcr.io/nylser/carla:leaderboard as carla
+FROM carlasim/carla:0.9.14 as carla
+# FROM ghcr.io/nylser/carla:leaderboard as carla
 
 # supply the base image with an environment supporting ROS UI via x11
 FROM osrf/ros:noetic-desktop-full-focal

 ```

### docker-compose.yml

```diff
diff --git a/build/docker-compose.yml b/build/docker-compose.yml
index d1ae1df..ef1b503 100644
--- a/build/docker-compose.yml
+++ b/build/docker-compose.yml
@@ -20,8 +20,8 @@ services:
   carla-simulator:
     command: /bin/bash CarlaUE4.sh -quality-level=Epic -world-port=2000 -resx=800 -resy=600
     # Use this image version to enable instance segmentation cameras: (it does not match the leaderboard version)
-    # image: carlasim/carla:0.9.14
-    image: ghcr.io/nylser/carla:leaderboard
+    image: carlasim/carla:0.9.14
+    # image: ghcr.io/nylser/carla:leaderboard
     init: true
     expose:
       - 2000
```

## Usage

To run the dataset generator, first the Carla Simulator has to be running:

Start the docker containr `leaderboard-2.0`.

You can then run the dataset generator by executing the following command in an attached shell:

```bash
python3 perception/src/dataset_generator.py --host carla-simulator --port 2000 --use-empty-world
```

With the `--use-empty-world` flag, the dataset generator will assume the Carla world to be empty and will spawn the ego
vehicle, as well as the traffic lights and the pedestrians manually. This is useful for debugging purposes.

After the dataset generator has been started, the Carla world will be populated with pedestrians and traffic lights and
the ego vehicle will be spawned and start driving around the map.

The dataset generator will record the images from the instance segmentation camera, as well as the corresponding rgb
camera images in four directions: `[center, left, right, back]`.

We then store these images in the output directory can be specified with the `--output-dir`
flag.

Specify the desired map via `--town` and use `TownXX` with XX being 01, 02, ..., 07, 10, 11, 12.
Default value is `Town12` like used in leaderboard 2.0.

### Using with leaderboard

To use the dataset generator with the leaderboard, first start the leaderboard evaluation by adjusting the command in
the
[docker-compose.yml](../../build/docker-compose.yml) to run the leaderboard evaluator:

  ```yaml
  agent:
    # ...
    command: bash -c "sleep 10 && python3 /opt/leaderboard/leaderboard/leaderboard_evaluator.py --debug=0 --routes=/opt/leaderboard/data/routes_devtest.xml --agent=/opt/leaderboard/leaderboard/autoagents/npc_agent.py --host=carla-simulator --track=SENSORS"
    # ...
```

Once the leaderboard evaluator is running, you can start the dataset generator in an attached shell:

```bash
python3 perception/src/dataset_generator.py --host carla-simulator --port 2000
```

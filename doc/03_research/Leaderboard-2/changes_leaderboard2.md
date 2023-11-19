# Overview leaderboard 2.0

**Summary:** New Features and changes made with the CARLA leaderboard-2.0

---

## Author

Samuel Kühnel

## Date

17.11.2023

## General Information

Leaderboard 1.0             |  Leaderboard 2.0
:-------------------------:|:-------------------------:
![leaderboard-1](../../00_assets/leaderboard-1.png)  |  ![leaderboard-2](../../00_assets/leaderboard-2.png)

As shown in the images above the new leaderboard seems to have way more traffic than the previous version. The leaderboard 2.0 uses an enhanced version of CARLA 0.9.14. So be aware that even if the documentation mentions this version tag, there are probably features missing.
Therefore it is recommended to use the latest version.

### Submissions

Every team can submit five times per month to the CARLA Leaderboard 2.0. The number of submissions is reset every month automatically. The full start guide and documentation can be found [here](https://leaderboard.carla.org/get_started/).

## New Features

### Maps

In the Leaderboard 2.0 new and larger maps were added, [town 12](https://carla.readthedocs.io/en/latest/map_town12/) and [town 13](https://carla.readthedocs.io/en/latest/map_town13/).
These maps consist of different regions e.g. downtown zone with high bulidngs arranged in blocks or rural areas. The city is surrounded by a highway system with 3-4 lanes and intersections/junctions.

The evaluation of submissions will be done on a secret map **town 14** that is kept secret but is designed similar to town 12 and 13.

### Scenarios and training database

The new leaderboard comes with a large set of (new) scenarios that the autonomous agent has to accomplish. Some examples are merging lanes, emergency vehicles or accidents. For each of the new scenrios CARLA provides logs from a 100% finish.

These logs ca be used by the _training database creation tool_. With this python script the user is able to do the test runs and collect sensor data. The user is able to specify a list of sensors, weather, destination folder for saved sensor data and the logs that will be used to do the run.

The script can be downloaded [here](https://leaderboard-logs.s3.us-west-2.amazonaws.com/capture_sensor_data.py).

The logs can be downloaded [here](https://leaderboard-logs.s3.us-west-2.amazonaws.com/Scenario+Logs.zip).

To use the script you can follow these steps:

1. Specify what sensors you want to use, e.g.:

    ```python
    SENSORS = [
        [
            'CameraTest',
            {
                'bp': 'sensor.camera.rgb',
                'image_size_x': 720, 'image_size_y': 1080, 'fov': 100,
                'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
            },
        ]
    ```

2. Specify the weather with the ```carla.WatherSettings``` class, e.g.:

    ```python
    WEATHER = carla.WeatherParameters(
        sun_azimuth_angle=-1.0, sun_altitude_angle=70.0,
        cloudiness=30.0, precipitation=0.0, precipitation_deposits=80.0, wetness=15.0,
        wind_intensity=10.0,
        fog_density=2.0, fog_distance=0.0, fog_falloff=0.0)
    ```

3. Specify what logs you want to use, e.g.:

    ```python
    RECORDER_INFO = [
    {
        'folder': "ScenarioLogs/Accident",
        'start_time': 20,
        'duration': 10
    }
    ```

4. Select destination folder for sensor data, e.g.:

    ```python
    DESTINATION_FOLDER = "database"
    ```

5. Start script with ```python capture_sensor_data.py```.

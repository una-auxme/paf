# How to submit to Leaderboard 2.0

**Summary:** This file contains a quick guide on how to prepare the submission for the carla Leaderboard

---

## Author

Samuel Kühnel

## Date

01.12.2023

## General Information

A complete guide can be found [here](https://leaderboard.carla.org/submit/). For the submission a special Dockerfile was created: ```Dockerfile_Submission```. This file contains the complete Dockerfile with all necessary settings.

Submissions can be seen [here](https://eval.ai/web/challenges/challenge-page/2098/my-submission).

### Environment variables

For the submission you need to set the following environment variables:

* ```${CARLA_ROOT}```: Leads to the carla root directory

* ```${SCENARIO_RUNNER_ROOT}```: Szenario runner root directory

* ```${LEADERBOARD_ROOT}```: Leaderboard root directory
* ```${TEAM_CODE_ROOT}```: Our agent code root directory (```/workspace/code```)
* ```${ROS_DISTRO}```: Used ROS distribution (```noetic```)

### Security controls

As a safeguard we added a command to the Dockerfile that deletes all files in `/workspace/code` that contain the string `import carla`. The usage of the carla API is forbidden in the Leaderboard 2.0 and could lead to disqualification.

### Submitting the image

To submit the image the python module `evalai` needs to be installed with `pip install evalai`.

After that the local PC needs to be authenticated with the evalai user token. For this you can use the command `evalai set_token <token>`.

To push the image the command `evalai push <image>:<tag> --phase <phase_name>` is used. `<phase_name>` specifies the Leaderboard track.

We will mostly use the command `evalai push <image>:<tag> --phase carla-leaderboard-20-map-2098 --private`

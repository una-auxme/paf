# Control Research PAF2024

**Summary:** This file discusses the question: Is there an autopilot in CARLA or elsewhere that can be used to improve our own control logic.

- [The Carla Traffic Manger](#the-carla-traffic-manger)

## The Carla Traffic Manger

You can find documentation on the Carla-Traffic-Management system under the following [here](https://carla.readthedocs.io/en/latest/adv_traffic_manager/).
In short there is a global planner, which can get accurate positions for each vehicle as well as optimize routes which might collide with each other. It's certainly a space for inspiration also for all other systems.

For control specifically the following link provides the PID controller for actuating steering throttle and brake control: [PID-Controller](https://carla.org/Doxygen/html/dc/d75/PIDController_8h_source.html).

Also, the included Constants-File is quite interesting: [Constants](https://carla.org/Doxygen/html/d1/d45/Constants_8h.html), because it provides PID-Values for the controller, which seem to work well with carla vhicles.

Maybe also related to the control scheme: [CARLA-PID-MoemenGaafar](https://github.com/MoemenGaafar/CARLA-PID-Controller)

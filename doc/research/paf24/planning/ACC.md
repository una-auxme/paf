# ACC

## General
The main goal of an ACC (Adaptive Cruise Control) is to follow a car driving in front while keeping a safe distance to it. This can be achieved by adjusting the speed to the speed of the car in front.
In general, classic ACC systems are designed for higher velocity (e.g. > 40 km/h). Apart from that, there are Stop & Go systems that support lower velocites (e.g. < 40 km/h). In our case, both systems might be needed and it might be reasonable to develop two different systems for ACC and Stop & Go.
There are basically three different techniques that can be used to implement an ACC: PID Control, Model Predictive Control and Fuzzy Logic Control. Another option is CACC (Cooperative Adaptive Cruise Control) but this is not relevant for our project since it requires communication between the vehicles.

### PID Control

### Model Predictive Control (MPC)
It calculates the current control action by solving an online, iterative and finite-horizon optimization of the model.
Procedure:
1. Prediction of future system states based on current states
2. Computation of the cost function for a finite time horizon in the future
3. Implementation of the first step of the solved control sequence
4. Application of the feedback control loop to compensate for the predictive error and model inaccuracy
5. Sampling of new current states and repitition of the process

### Fuzzy Logic Control (FLC)
Provides a unified control framework to offer both functions: ACC and Stop & Go.


![Example of a FLC control algorithm](../../../assets/research_assets/ACC_FLC_Example_1.png)

## Discussion

- When should the ACC be used? Only when driving straight forward?
- How to test adaptations of the ACC? Create test scenarios?
- Which output should be transfered to the Acting component? Only the desired speed?
- Which input do we get from the Perception component? The distance and velocity of the car in front?

## ACC Algorithm

### Current implementation

- General behaviour:
    checks for obstacle distance < safety distance
  
    calculates new speed based on obstacle distance
  
    else keep current speed
  
- publishes speed to acc_velocity
- safe distance calculation is currently not correct, uses speed +  (speed * 0.36)² which results in wrong distances
- if car in front of us ignores speed limits we ignore them as well
- some parts of unstuck routine are in ACC and need to be refactored
- same goes for publishing of current waypoint, this should not be in ACC

In summary, the current implementation is not sufficient and needs major refactoring.

### Concept for new implementation

The new concept for the ACC is to take the trajectory, look at all or a limited subset of the next points and add a target velocity and the current behaviour to each point.
This way the Acting has more knowledge about what the car is doing and can adjust accordingly in a local manner.
For this a new trajectory message type was implemented in #511.

![trajectoryMsg](https://github.com/user-attachments/assets/0b452f1a-4c60-45b2-882f-3a50118c9cb9)

Since behaviour is passed as an ID a new enum for behaviours was implemented in utils.py as well.

The general idea for speeds above the 40 km/h mark is to calculate a proper safety distance and a general target velocity. For speeds lower than that a stop and go system needs to be discussed.

For safety distance the old approach can simply be modified. For example, (speed / 10)*3 + (speed / 10)² is a well known formula for calculating the braking distance.

For a general speed target we need to take the speed and the speed of the car in front into account. In cases where the car in front is substantially slower than the speed limit ACC could inititate overtaking.

With the general speed target and the current distance to the car in front we can calculate the target velocity for each point in the trajectory up to the car in front for example by interpolation. Points further that that will be inititalized with the speed limit at that position.

### Possible next steps
A seperate file for the new ACC should be created to not disturb the system.

The parts that might get cut from ACC like current waypoint and unstuck routine need to be evaluated for necessity and if need be moved to somewhere more fitting.

Implement publisher for new message type.

Start implementing safety distance and general target speed logic. Subscriber logic could be taken from old implementation.

### Requirements:
- obstacle speed
- obstacle distance



## Sources
https://www.researchgate.net/publication/335934496_Adaptive_Cruise_Control_Strategies_Implemented_on_Experimental_Vehicles_A_Review

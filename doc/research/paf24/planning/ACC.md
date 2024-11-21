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




## Sources
https://www.researchgate.net/publication/335934496_Adaptive_Cruise_Control_Strategies_Implemented_on_Experimental_Vehicles_A_Review
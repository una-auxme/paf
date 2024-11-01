# Driving Score Computation

**Summary:** The Driving score is the main performance metric of the agent and therefore has to be examined.

- [A Starting Point for Metrics and Measurements](#a-starting-point-for-metrics-and-measurements)
- [Driving score](#driving-score)
- [Route completion](#route-completion)
- [Infraction penalty](#infraction-penalty)
- [Infractions](#infractions)
- [Off-road driving](#off-road-driving)
- [Additional Events](#additional-events)
- [Sources](#sources)

## A Starting Point for Metrics and Measurements

The CARLA Leaderboard sets a public example for evaluation and comparison.
The driving proficiency of an agent can be characterized by multiple metrics. For this leaderboard, the CARLA team selected a set of metrics that help understand different aspects of driving.

## Driving score

$\frac{1}{N}\sum^i_N R_i P_i$

- The main metric of the leaderboard, serving as an aggregate of the average route completion and the number of traffic infractions. Here $N$ stands for the number of routes, $R_i$ is the percentage of completion of the $i$-th route, and $P_i$ is the infraction penalty of the $i$-th route.

## Route completion

$\frac{1}{N}\sum^i_N R_i$

- Percentage of route distance completed by an agent, averaged across $N$ routes.

## Infraction penalty

$\prod_j^{ped, veh, ... stop} (p_j^i)^{n_{infractions}}$

- Aggregates the number of infractions triggered by an agent as a geometric series. Agents start with an ideal 1.0 base score, which is reduced by a penalty coefficient for every instance of these.

## Infractions

The CARLA leaderboard offers individual metrics for a series of infractions. Each of these has a penalty coefficient that will be applied every time it happens. Ordered by severity, the infractions are the following.

- Collisions with pedestrians $0.50$
- Collisions with other vehicles $0.60$
- Collisions with static elements $0.65$
- Running a red light $0.70$
- Running a stop sign $0.80$

Some scenarios feature behaviors that can block the ego-vehicle indefinitely. These scenarios will have a timeout of 4 minutes after which the ego-vehicle will be released to continue the route. However, a penalty is applied when the time limit is breached

- Scenario Timeout $0.70$

The agent is expected to maintain a minimum speed in keeping with nearby traffic. The agent’s speed will be compared with the speed of nearby vehicles. Failure to maintain a suitable speed will result in a penalty.
The penalty applied is dependent on the magnitude of the speed difference, up to the following value:

- Failure to maintain speed $0.70$

The agent should yield to emergency vehicles coming from behind. Failure to allow the emergency vehicle to pass will incur a penalty:

- Failure to yield to emergency vehicle $0.70$

Besides these, there is one additional infraction which has no coefficient, and instead affects the computation of route completion $(R_i)$.

## Off-road driving

If an agent drives off-road, that percentage of the route will not be considered towards the computation of the route completion score.

## Additional Events

Some events will interrupt the simulation, preventing the agent to continue.

- Route deviation
If an agent deviates more than $30$ meters from the assigned route.
- Agent blocked
If an agent is blocked in traffic without taking any actions for $180$ simulation seconds.
- Simulation timeout
If no client-server communication can be established in $60$ seconds.
- Route timeout
This timeout is triggered if the simulation of a route takes more than the allowed time. This allowed time is computed by multiplying the route distance in meters by a factor of $0.8$.

## Sources

- [Alpha Drive Carla Leaderboard Case Study](https://alphadrive.ai/industries/automotive/carla-leaderboard-case-study/)
- [Carla Leaderboard](https://leaderboard.carla.org/)

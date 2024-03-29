# Decision-making module

**Summary:** This page gives a brief summary over possible decision-making choices their ad- and disadvantages as well as the opportunity to interchange them later on. Also, possible implementation options for those concepts are given.

---

## Author

Josef Kircher

## Date

01.12.2022

## Prerequisite

---
<!-- TOC -->
* [Decision-making module](#decision-making-module)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Decision-making algorithms](#decision-making-algorithms)
    * [Finite State machine](#finite-state-machine)
    * [Markov Chain](#markov-chain)
    * [Decision Tree](#decision-tree)
  * [Previous approaches](#previous-approaches)
    * [PAF21-1](#paf21-1)
    * [PAF21-2](#paf21-2)
    * [PSAF1 2020](#psaf1-2020)
    * [PSAF2 2020](#psaf2-2020)
  * [Python or ROS libraries for these decision-making algorithms](#python-or-ros-libraries-for-these-decision-making-algorithms)
    * [State machines](#state-machines)
      * [SMACH](#smach)
      * [SMACC](#smacc)
    * [Markov Chains](#markov-chains)
      * [QuantEcon](#quantecon)
      * [markov_decision_making](#markov_decision_making)
    * [Decision trees](#decision-trees)
      * [pytrees](#pytrees)
  * [Conclusion](#conclusion)
    * [Sources](#sources)
<!-- TOC -->

## Decision-making algorithms

### Finite State machine

A finite-state machine (FSM) or finite-state automaton (FSA, plural: automata), finite automaton, or simply a state machine, is a mathematical model of computation.
It is an abstract machine that can be in exactly one of a finite number of states at any given time.
The FSM can change from one state to another in response to some inputs; the change from one state to another is called a transition.
An FSM is defined by a list of its states, its initial state, and the inputs that trigger each transition.
Finite-state machines are of two types—deterministic finite-state machines and non-deterministic finite-state machines. A deterministic finite-state machine can be constructed equivalent to any non-deterministic one.

#### Advantages

* easy to implement
* we know most of the scenarios (finite state space)
* previous groups have solutions we could adapt/extend

#### Disadvantages

* many states necessary
* even though we can try to map all possible states, there still might be some situation we could not account for

### Markov Chain

A Markov chain or Markov process is a stochastic model describing a sequence of possible events in which the probability of each event depends only on the state attained in the previous event.
A countably infinite sequence, in which the chain moves state at discrete time steps, gives a discrete-time Markov chain. A continuous-time process is called a continuous-time Markov chain. It is named after the Russian mathematician Andrey Markov.

#### Advantages

* possible to build Markov Chain from State machine
* experience from previous projects
* only depends on current state ("memorylessness")

#### Disadvantages

* might be complicated to implement
* probabilities for transitions might need to be guessed, empirically estimated

### Decision Tree

A decision tree is a decision support tool that uses a tree-like model of decisions and their possible consequences, including chance event outcomes, resource costs, and utility.
It is one way to display an algorithm that only contains conditional control statements. Decision trees are commonly used in operations research, specifically in decision analysis, to help identify a strategy most likely to reach a goal, but are also a popular tool in machine learning.

#### Advantages

* easy implementation
* tree like structure usable in Machine Learning (Random Forest e.g.)

#### Disadvantages

* multiple decision trees necessary
* prediction independent of previous state

## Previous approaches

### PAF21-1

#### State machine

* 2 state machines: one for maneuvers, one for speed control
* Speed control more complex, when to brake seems like the most challenging task

#### Take away

* Some states seem to be comparable to what we are required to accomplish by the leaderboard
* Our task might be more complex, needs additional states and transitions
* I'm uncertain about an extra speed state, might be easier to handle that more locally by the local planner, maybe in combination with an observer element that keeps track of the surrounding by processing the information from `Perception`

### PAF21-2

#### No clear concept

* some sort of state machine integrated in local planner
* obstacle planner for dynamic obstacles (pedestrians, cars, bicycles)
* useful parameters which we could adapt
* path prediction for obstacles
* obstacles are only interesting if they cross the path of the ego vehicle

#### Take away

* Obstacle planner might be useful for dynamic obstacle detection if not handled elsewhere
* path prediction might reduce the number objects tracked that we could interfere with
* Also, if we adapt our local plan this path prediction of other vehicles might come in handy
* On the other hand, overhead to keep track of vehicles and maybe repredict paths if some vehicles change direction

### PSAF1 2020

#### State machine

* Three driving functions: Driving, stopping at traffic light, stopping at stop sign
* First project iteration so state machine more simple
* still covers many important scenarios

#### Take away

* Good starting point to have a minimal viable state machine
* Need adaption depending on what information we are getting forwarded/process in the planning module

### PSAF2 2020

#### Decision tree

* This team used a decision tree to cover the major driving scenarios
* Within the scenarios the actions are more linear
* Reminds me of the execution of a state where driving scenarios are the states and the execution the things our local planner should do within that state

#### Take Away

* Even though the approach is different, the execution might be similar to the other team algorithms
* We might not be interested in a decision tree as we want to keep the option to switch to a Markov chain, which would be more overhead if we start with a decision tree

## Python or ROS libraries for these decision-making algorithms

### State machines

#### SMACH

* Task-level architecture for creating state machines for robot behaviour.
* Based on Python
* Fast prototyping: Quickly create state machines
* Complex state machines can easily be created
* Introspection: smach_viewer provides a visual aid to follow the state machine executing its tasks
  * smach_viewer is unmaintained and does not work with noetic
* Allows nested state machines
* Values can be passed between states
* Tutorials and documentation seems to be easy to understand so creating a first state machine shouldn't be too hard
* working with several ROS topics and messages within the state machine needs to be evaluated:
  * the execution of states is mostly planned to happen in the local planner so for just sending a ROS message, SMACH might be efficient

Not use SMACH for:

* Unstructured tasks: SMACH is not efficient in sheduling unstructured tasks
* Low-level systems: SMACH is not build for high efficiency, might fall short for emergency maneuvers

* Simple examples run without problem

#### SMACC

* event-driven, asynchronous, behavioral state machine library
* real-time ROS applications
* written in C++
* designed to allow programmers to build robot control applications for multicomponent robots, in an intuitive and systematic manner.
* well maintained, lots of prebuild state machines to possibly start from

Why not use SMACC:

* might get some time to get back into C++
* more sophisticated library might need more time to get used to
* awful country music in the back of tutorial videos

* Tutorials do not run without further debugging which I didn't invest the time to do so

### Markov Chains

#### QuantEcon

* a economics library for implementing Markov chains
* more focussed on simulation than actually using it in an AD agent
* maybe usable for testing and simulating a Markov chain before implementing it

#### markov_decision_making

* ROS library for robot decision-making based on Markov Decision Problems
* written in C++
* callback-based action interpretation allows to use other frameworks (SMACH)
* relatively easy to implement hierarchical MDPs
* supports synchronous and asynchronous execution

Why not use markov_decision_making:

* not maintained
* only works with ROS hydro

### Decision trees

#### pytrees

* easy framework for implementing behaviour trees
* written in python
* used by a group two years ago
* not usable for real-time application code according to their docs
* priority handling - higher level interrupts are handled first

## Conclusion

In my opinion, a state machine would be a great start for the project. There are plenty of resources available from recent projects.
It needs to be further discussed if the libraries presented above possess the needed functionality to run our state machine. The planning team might meet on the issue and present a suitable solution.
It is possible to start with a skeleton of both and compare them.

### Sources

<https://de.wikipedia.org/wiki/Markow-Kette#:~:text=Eine%20Markow%2DKette%20(englisch%20Markov,das%20Eintreten%20zuk%C3%BCnftiger%20Ereignisse%20anzugeben.>

<https://github.com/ll7/paf21-1/wiki/Decision-Making-Component>

<https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_planning>

<https://github.com/ll7/psaf1/tree/master/psaf_ros/psaf_local_planner>

<https://github.com/ll7/psaf2/tree/main/Planning/behavior_agent>

<http://wiki.ros.org/smach>

<http://wiki.ros.org/smach_viewer>

<https://automaticaddison.com/how-to-create-a-finite-state-machine-using-smach-and-ros/>

<https://quantecon.org/quantecon-py/>

<https://python.quantecon.org/finite_markov.html>

<http://wiki.ros.org/markov_decision_making>

<https://py-trees.readthedocs.io/en/devel/introduction.html>

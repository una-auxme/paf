# Re-evaluation of decision making component

**Summary:** This page gives a foundation for the re-evaluation of the decision-making

---

## Author

Josef Kircher

## Date

26.01.2023

## Prerequisite

---
<!-- TOC -->
* [Re-evaluation of decision making component](#re-evaluation-of-decision-making-component)
  * [**Summary:** This page gives a foundation for the re-evaluation of the decision-making](#summary-this-page-gives-a-foundation-for-the-re-evaluation-of-the-decision-making)
  * [Author](#author)
  * [Date](#date)
  * [Prerequisite](#prerequisite)
  * [Reasons for re-evaluation](#reasons-for-re-evaluation)
  * [Options](#options)
    * [Pylot](#pylot)
    * [Pytrees](#pytrees)
      * [Pros](#pros)
      * [Cons](#cons)
  * [Conclusion](#conclusion)
    * [Sources](#sources)
<!-- TOC -->
## Reasons for re-evaluation

In the last sprint, I tried to get a graphic tool to work with the docker container withing the project. That failed, but I still think, that a graphical representation would be helpful.
Other reasons are:

* not much time has been allocated for the state machine so far
* using SMACH would result in a mostly from scratch implementation
* harder to debug due to the lack of a graphic representation

## Options

### Pylot

Uses a simple state machine, mostly working with loggers, the skeleton of the state machine might be insteresting,
as there a different states for preparing and executing a lane change, but overall it might be more interesting for the local planner as most of the interesting stuff is centered around obstacle evasion and avoidance.

### Pytrees

In the last sprint review, Lennart gave me a hint to look at the solution of a former team that used the decision tree library [pytrees](https://py-trees.readthedocs.io/en/devel/index.html).

It is very similar to a finite state machine, so I further investigated both the code of the previous team and the library with its documentation as well.

As it is looking very promising, I list here a few arguments to help support my decision.

#### Pros

* support a graphical representation at runtime with rqt
* a lot of similar driving scenarios as the old team
* so a lot of code can be recycled
* quite intuitive and easy to understand
* only a limited amount of commands (easy to learn)
* well documented
* maintained

#### Cons

* only a couple of decision can be made inside the tree, so it might be more complicated to depict the complex behaviour of the ego vehicle
* A lot of time was invested in the design of the original state machine, might be needed to be adapted

## Conclusion

Pylot doesn't contain a usable alternative for having a graphical solution for debugging, however the approach of the project looks promising and might be considered in the process of this project.

Pytrees in itself looks more promising, it was already used in an older project to inserting it in the docker environment and the project should not be too hard, also the old solution could be adapted and extended.
It is also easier to have a working starting point than to start from scratch. It will be easier to debug and as I have no experience with working with `SMACH` or `pytrees` the incorporation process will be equal.
This will result in a change of the approach of the decision making component. However, both approaches are related, so the porting of the design should not be too time intensive.

After completing this reevaluation process, I plead to use pytrees for this project.

### Sources

<https://github.com/erdos-project/pylot>

<https://py-trees.readthedocs.io/en/devel/index.html>

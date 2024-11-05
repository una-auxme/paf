# Research about the existing architecture documentation

The repository already holds various documents about how the architecture was planned and how the architecture should
look or is
looking. As it is a crucial part of the project to understand the component interactions, especially the up-to-date
version of it, in this document I will give a brief overview of the existing documentation and some links to the
up-to-date versions.

## Existing architecture documentation from the previous semester

The main architecture documentation can be found [here](/doc/general/architecture.md).
It contains information to most nodes and what they subscribe / publish.

A miro board with the existing architecture (Perception, planning, acting) is existing.
![Architecture overview](/doc/assets/overview.jpg)
The miro-board can be
found [here](https://miro.com/welcomeonboard/a1F0d1dya2FneWNtbVk4cTBDU1NiN3RiZUIxdGhHNzJBdk5aS3N4VmdBM0R5c2Z1VXZIUUN4SkkwNHpuWlk2ZXwzNDU4NzY0NTMwNjYwNzAyODIzfDI=?share_link_id=785020837509).

This miro board does contain the main architecture details and flows of information but when comparing it to the
rosgraph of the nodes and topics it seems like the diagram is not complete.

### Current Rosgraph of the nodes and topics of the project

[//]: # "![Up to date ros graph](/doc/assets/research_assets/rosgraph.svg)"
![Up to date ros graph](/doc/assets/research_assets/rosgraph_leaf_topics.svg)

### Rewritten rosgraph divided into perception, planning and acting

![RosGraphDrawIO](/doc/assets/research_assets/node_path_ros.png)

There you can see the nodes and the in- and outputs. In represents the subscribed topics and out the published topics.
This should now be extended by what logic is happening in which node and how the nodes are connected.

## Perception architecture

- Extended information of how the perception works can be found [here](/doc/perception/README.md)

## Planning architecture

- Extended information of how the planning works can be found [here](/doc/planning/README.md)

## Acting architecture

- Extended information of how the acting works can be found [here](/doc/acting/architecture_documentation.md)

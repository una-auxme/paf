# Relation between planning and acting

This diagram shows the basics of the communication between the planning (green) and acting (blue) components. The information is gained using the rqt-graph. There are three main topics which represent the communication:

- `/paf/hero/trajectory`
- `/paf/hero/emergency`
- `/paf/hero/curr_behavior`

![Diagram showing the involved nodes and topics and their relation](../../../assets/research_assets/planning_acting_communication.svg)

## Useful commands

- The types of the messages published on the mentioned topics can be found out with the command `rostopic type <topic>`.
- To have a deeper look on the message types, you can use the command `rosmsg show <messageType>`.
- `rostopic echo <topic>` shows all the messages published on the specified topic.

## Messages

The topic `/paf/hero/trajectory` uses the message type `nav_msgs/Path`. This type looks as follows:

![Format of Path messages](../../../assets/research_assets/nav_msgs_Path_type.png)

Every message that is published consists of several parts of this structure. One concrete example of a part of such a message can be seen here:

![Example of a Path message](../../../assets/research_assets/trajectory_example.png)

The topic `/paf/hero/emergency` uses the message type `std_msgs/Bool`, so the emergency case can either be true or false.

The topic `/paf/hero/curr_behavior` uses the message type `std_msgs/String`. Examples of these messages can be seen here:

![Examples of messages posted on the topic curr_behavior](../../../assets/research_assets/curr_behavior_example.png)

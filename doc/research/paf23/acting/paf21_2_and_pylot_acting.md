# PAF Research: Robert Fischer

**Summary:** This page contains the research into the action component of the PAF21_2 group and pylot.

- [PAF Research: Robert Fischer](#paf-research-robert-fischer)
  - [Acting](#acting)
    - [List of Inputs/Outputs](#list-of-inputsoutputs)
    - [Challenges](#challenges)
  - [PAF21\_2 Acting](#paf21_2-acting)
    - [Standardroutine](#standardroutine)
    - [Unstuck-Routine](#unstuck-routine)
    - [Deadlock](#deadlock)
    - [Verfolgung von Hindernissen](#verfolgung-von-hindernissen)
    - [Messages](#messages)
    - [StanleyController](#stanleycontroller)
    - [PID Controller](#pid-controller)
    - [Emergency Modus](#emergency-modus)
    - [Bugabuses](#bugabuses)
  - [Pylot Acting (Control)](#pylot-acting-control)
  - [Control Types](#control-types)
    - [PID](#pid)
    - [MPC](#mpc)
    - [Carla\_Autopilot](#carla_autopilot)
  - [Basic Cotrol Code](#basic-cotrol-code)
    - [**control\_eval\_operator.py**](#control_eval_operatorpy)
    - [**messages.py**](#messagespy)
    - [**pid.py**](#pidpy)
    - [**pid\_control\_operator.py**](#pid_control_operatorpy)
    - [**utils.py**](#utilspy)
  - [MPC Control Code](#mpc-control-code)

## Acting

- Longitudinal control
  - PID controller
- Lateral control
  - Pure Pursuit controller

    ![Untitled](../../assets/research_assets/pure_pursuit.png)

  - Stanley controller

    ![Untitled](../../assets/research_assets/stanley_controller.png)

### [List of Inputs/Outputs](https://github.com/una-auxme/paf/blob/main/doc/research/acting/acting_implementation.md#list-of-inputsoutputs)

- Subscribes to:
  - [nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) : to get the current position and heading
  - [nav_msgs/Path Message](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) : to get the current trajectory
  - emergency breaking msg : to initiate emergency breaking
  - speed limit msg : to get the maximum velocity
- Publishes:
  - [CarlaEgoVehicleControl.msg](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/#carlaegovehiclecontrolmsg) : to actually control the vehicles throttle, steering

### [Challenges](https://github.com/una-auxme/paf/blob/main/doc/research/acting/acting_implementation.md#challenges)

A short list of challenges for the implementation of a basic acting domain and how they these could be tackled based on the requirements mentioned above.

- The vehicle needs to know its own position => [nav_msgs/Odometry Message](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) or [GNSS](https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor) sensor
- The vehicle needs to know its own velocity => can be calculated from last/current position and time or the [speedometer](https://leaderboard.carla.org/#map-track) pseudosensor can be used
- The vehicle needs to know its planned trajectory => [nav_msgs/Path Message](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html) this trajectory may need to be updated to accommodate obstacles
- Longitudinal control => a simple PID controller should suffice
- lateral control => Pure Pursuit as well as Stanley controller should be implemented, following tests can show, where to use each controller.
- additional features:
  - emergency breaking => this command is supposed to bypass longitudinal and lateral controllers (and should use the bug discoverd by [paf21-2](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#bugabuses))
  - additional functionality mostly should be added here .

## PAF21_2 Acting

### [Standardroutine](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#standardroutine)

![Untitled](../../assets/research_assets/standard_routine_paf21_2.png)

- Longitudinal control
  - PID controller
- Lateral control
  - Stanley controller

 → CarlaEgoVehicleControl-Message an Carla

### [Unstuck-Routine](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#unstuck-routine)

Timer und Schwellenwerte um Stuck Situation zu erkennen

→ paar meter rückwärts ohne lenken

- **Funktionsweise:**
    1. **Initialisierung:**
        - Timer wird auf null gesetzt.
    2. **Überprüfung der Fahrzeugbewegung:**
        - Aktuelle Geschwindigkeit wird mit der gewünschten Zielgeschwindigkeit verglichen.
        - Falls aktuelle Geschwindigkeit unter einem Schwellenwert liegt und Zielgeschwindigkeit darüber, wird der Timer gestartet.
    3. **Timer-Überwachung:**
        - Timer wird verwendet, um die festgefahrene Situation zu behandeln.
        - Timer-Wert ist entscheidend und wird durch Schwelle definiert.
    4. **Unstuck-Routine-Auslösung:**
        - Wenn Timer einen bestimmten Wert erreicht, wird die Unstuck-Routine aktiviert.
        - Actor fährt einige Meter rückwärts, ohne zu lenken.
    5. **Rückkehr zum Normalbetrieb:**
        - Nach erfolgreicher Anwendung der Unstuck-Routine kehrt das Fahrzeug zum normalen Betrieb zurück.
- **Notwendige Überlegungen:**
  - Schwelle ist entscheidend, um sicherzustellen, dass das Fahrzeug nicht aufgrund minimaler Geschwindigkeitsunterschiede als festgefahren betrachtet wird.
  - Timer stellt sicher, dass die Unstuck-Routine nicht sofort ausgelöst wird, sondern nur nach einer vordefinierten Zeit.

### [Deadlock](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#deadlock)

- **Herausforderung:**
  - Mögliche Situationen, in denen der Actor aufgrund von Unstimmigkeiten in der Planung und Perception in einer festgefahrenen Lage endet.
- **Ursachen für Deadlocks:**
  - Unstimmigkeiten zwischen der Planung der Perception, insbesondere bei Interaktionen mit anderen Verkehrsteilnehmern.
  - Unvorhergesehene Ereignisse, die nicht durch Standard- oder Unstuck-Routine gelöst werden können.
- **Perception-Paket:**
  - Verantwortlich für sichere Durchquerung durch Sensoren, insbesondere Hinderniserkennung.
  - Kann Unstimmigkeiten mit der Planung verursachen.
- **Plannning-Paket:**
  - Plant optimale Routen unter Berücksichtigung der besten und schnellsten Wege.
  - Kann zu unverhofftem Verhalten führen, wie dem Überqueren der Gegenfahrbahn.
- **Deadlock-Szenarien:**
  - Zyklische Wartesituationen, z. B. wenn ein Fahrzeug auf der Gegenfahrbahn wartet und gleichzeitig auf uns wartet.
- **Deadlock-Vermeidungsstrategie:**
  - Zustandsmaschine als Lösung, unabhängig von anderen Paketen.
  - Laufender Timer überwacht die Dauer, in der sich das Fahrzeug nicht bewegt.
  - Überschreitung eines Schwellwerts führt zur Aktivierung der Deadlock-Routine.
- **Deadlock-Routine:**
  - Stoppt alle Aktivitäten des Actors.
  - Führt schrittweise Manöver durch: Rückwärtsfahren, Warten, Vorwärtsfahren.
  - Ziel ist die Befreiung aus festgefahrenen Situationen.
- **Optimierung der Zeitparameter:**
  - Kritisch, um unnötige Deadlock-Routine-Auslösungen zu vermeiden.
  - Zu kurze Zeiten können zu falschen Deadlock-Erkennungen führen.
  - Zu lange Zeiten können wertvolle Zeit bei der Zielerreichung kosten.
- **Zusammenfassung:**
  - Die Deadlock-Vermeidungsstrategie basiert auf einer intelligenten Zustandsmaschine und einem Timer, um das Fahrzeug aus komplexen oder simpleren festgefahrenen Situationen zu befreien.
  - Die Optimierung der Zeitparameter ist entscheidend für die Wirksamkeit der Deadlock-Vermeidung, um unnötige Unterbrechungen zu verhindern und gleichzeitig effizient zum Ziel zu gelangen.

### [Verfolgung von Hindernissen](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#verfolgung-von-hindernissen)

- **Herausforderungen:**
  - Zahlreiche Verkehrsteilnehmer erfordern angepasstes Verhalten für sichere Fahrt.
- **Vorteilhaftes Verhalten:**
  - **Bleibe in der Mitte der Spur:**
    - Zentrale Positionierung für optimale Sicherheit.
  - **Fahre niemandem auf:**
    - Vermeide Kollisionen durch sicheres Abstandhalten.
  - **Behalte die Fußgänger im Auge:**
    - Aufmerksamkeit auf Fußgänger, um Kollisionen zu vermeiden.
  - **Passe dich anderen an:**
    - Flexibles Verhalten gegenüber anderen Verkehrsteilnehmern.
  - **Sei kein Bully:**
    - Vermeide aggressives Verhalten gegenüber anderen Fahrzeugen oder Fußgängern.
- **Message-Format (PafObstacleFollowInfo.msg):**
  - **speed (float32):**
    - Aktuelle Geschwindigkeit des Actors.
  - **distance (float32):**
    - Entfernung zu einem Hindernis vor dem Fahrzeug.
  - **no_target (bool):**
    - Signalisiert, ob ein Hindernis vor dem Fahrzeug ist (False) oder ob das Ziel verschwunden ist (True).
  - **is_vehicle (bool):**
    - Kennzeichnet, ob das Hindernis ein Fahrzeug ist.
- **Umsetzung des Verhaltens:**
  - **Hindernisvorhanden (no_target=False):**
    - Geschwindigkeit anpassen und Pfad weiterverfolgen.
  - **Ziel verschwunden (no_target=True):**
    - Rückkehr zum Normalbetrieb.
  - **Entfernung zum Hindernis (distance):**
    - Ignorieren, wenn zu weit entfernt.
    - Abstand halten, wenn gefährlich nahe.
  - **Anpassung der Geschwindigkeit (speed):**
    - Gleiche Geschwindigkeit wie das Hindernis, um sicher zu folgen.
    - Kontrollierte Annäherung, wenn notwendig etwas schneller als das Hindernis.
- **Spezialfall - Fußgänger (is_vehicle=False):**
  - Vermeidung von rücksichtslosem Verhalten.
  - Physik-bedingte unvorhersehbare Situationen verhindern.
  - Kontrollierte Annäherung, um das Ziel erfolgreich zu erreichen.
  - Beachtung der Sicherheitsregeln, um Fahrzeugfunktionalität zu erhalten.

### [Messages](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#messages)

![Untitled](../../assets/research_assets/messages_paf21_2.png)

### [StanleyController](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#stanleycontroller)

- **Zuständigkeit:**
  - Bestimmung des Lenkeinschlags für autonomes Fahren.
- **Berechnung von Heading Error und Crosstrack Error:**
  - Basierend auf dem Paper "[Comparison of lateral controllers for autonomous vehicle: experimental results](https://hal.archives-ouvertes.fr/hal-02459398/document)".
  - **Heading Error:**
    - Maß für Abweichung der Ausrichtung des Fahrzeugs von der gewünschten Richtung.
  - **Crosstrack Error:**
    - Maß für seitliche Abweichung des Fahrzeugs von der geplanten Fahrspur.
- **Implementierung:**
  - Übernommen von [Gruppe 2](https://github.com/ll7/psaf2/wiki/Path-Tracking-Algorithmen#stanley-methode) aus dem Vorjahr.
  - Dokumentation der Implementierung vorhanden.
- **Eignung für Geschwindigkeiten:**
  - Sehr gut für niedrige und mittlere Geschwindigkeiten beim Vorwärtsfahren.
  - Bei hohen Geschwindigkeiten (>100) neigt das Fahrzeug zum Wobbeln.
  - Rückwärtsfahren ist möglich, aber wurde nicht optimal eingebaut.
- **Rückwärtstests:**
  - Testinformationen für das Rückwärtsfahren verfügbar, aber nicht von uns benötigt.
  - Rückwärtsfahren nicht als Hauptanwendungsfokus.
- **Optimierungspotenzial:**
  - Potenzielle Optimierungen könnten erforderlich sein, um Wobbeln bei hohen Geschwindigkeiten zu reduzieren.
  - Rückwärtsfahren könnte bei Bedarf weiter optimiert werden.

### [PID Controller](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#pid-controller)

- PID Controller ist dafür zuständig, festzulegen, wie stark Gas gegeben und gebremst wird
- nahezu Gruppe 2 Implementierung

### [Emergency Modus](https://github.com/ll7/paf21-2/tree/main/paf_ros/paf_actor#emergency-modus)

- **Zweck:**
  - Sofortige Vollbremsung im Notfall.
- **Auslösung:**
  - Nachricht von Planning oder Perception an den Actor signalisiert einen Notfall.
  - Actor führt eine Vollbremsung durch, um schnell zum Stillstand zu kommen.
- **Ende des Notfalls:**
  - Vollbremsung wird gestoppt, wenn eine Nachricht den Notfall als aufgelöst erklärt.
- **Kommunikation zwischen Komponenten:**
  - Planning oder Perception sendet Nachricht an den Actor, um den Notfall zu signalisieren.
  - Nachfolgende Nachricht informiert darüber, dass der Notfall vorbei ist.
- **Automatische Beendigung des Emergency Modus:**
  - Vollbremsung dauert an, bis eine Nachricht den Notfall als aufgelöst erklärt.

Schnellstes Bremsen durch Testing festgestellt [HIER](https://github.com/ll7/paf21-2/blob/main/docs/paf_actor/backwards/braking.md)

### Bugabuses

- Beim Bremsen wird der Rückwärtsgang eingelegt und Gas gegeben.
- Außerdem der Lenker im Falle der Notbremse maximal eingeschlagen, um den Bremsweg zu verringern. Dies führt dazu, dass das Auto nach dem Stillstand weiterhin seitlich abdriftet, weshalb die Notbremse dann wieder gelöst werden muss.

Tipp: Beim Einparken kann die Notbremse helfen, um seitlich in die Lücke zu rutschen ;)

## Pylot Acting (Control)

[pylot.control package — Pylot 0.2 documentation](https://pylot.readthedocs.io/en/latest/pylot.control.html#subpackages) doesnt show any useful documentation

[Control — Pylot 0.2 documentation](https://pylot.readthedocs.io/en/latest/control.html) same; doesnt show useful documentation

## Control Types

### PID

Follows waypoints using a PID controller.

implements a longitudinal and lateral controller

### MPC

- Utilizes model predictive control for speed and waypoint following.
- Predicts future states using a kinematic model to optimize control inputs.
- Parameters include mpc_horizon, mpc_steps, and mpc_weights

![Untitled](../../assets/research_assets/mpc.png)

• cost function can be designed to account for driving comfort

### Carla_Autopilot

- Uses CARLA auto pilot for driving on predefined routes.
- Independent of the planning component, relying on CARLA agent for navigation.
- Useful for testing components like perception or prediction without control concerns.

## Basic Cotrol Code

### **[control_eval_operator.py](https://github.com/erdos-project/pylot/blob/master/pylot/control/control_eval_operator.py)**

"""Operator that computes the accuracy metrics using reference waypoints
and the achieved waypoints.

Args:
    pose_stream (:py:class:`erdos.ReadStream`): The stream on which the
        vehicle transform is received.
    waypoints_stream (:py:class:`erdos.ReadStream`): The stream on which
        the waypoints are received from the planner.
    flags (absl.flags): Object to be used to access absl flags.
"""

### **[messages.py](https://github.com/erdos-project/pylot/blob/master/pylot/control/messages.py)**

"""This class represents a message to be used to send control commands.

Args:
    steer (:obj:`float`): Steer angle between [-1.0, 1.0].
    throttle (:obj:`float`): Throttle command between [0.0, 1.0].
    brake (:obj:`float`): Brake command between [0.0, 1.0].
    hand_brake (bool): Boolean controlling hand-brake engagement.
    reverse (bool): Boolean controlling reverse gear engagement.
    timestamp (:py:class:`erdos.timestamp.Timestamp`): The timestamp of
        the message.
"""

### **[pid.py](https://github.com/erdos-project/pylot/blob/master/pylot/control/pid.py)**

This module implements a longitudinal and lateral controller

### **[pid_control_operator.py](https://github.com/erdos-project/pylot/blob/master/pylot/control/pid_control_operator.py)**

Operator that uses PID to follow a list of waypoints.

The operator waits for the pose and waypoint streams to receive a watermark
message for timestamp t, and then it computes and sends a control command.

Args:
    pose_stream (:py:class:`erdos.ReadStream`): Stream on which pose
        info is received.
    waypoints_stream (:py:class:`erdos.ReadStream`): Stream on which
        :py:class:`~pylot.planning.messages.WaypointMessage` messages are
        received. The operator receives waypoints from the planning
        operator, and must follow these waypoints.
    control_stream (:py:class:`erdos.WriteStream`): Stream on which the
        operator sends :py:class:`~pylot.control.messages.ControlMessage`
        messages.
    flags (absl.flags): Object to be used to access absl flags.

### **[utils.py](https://github.com/erdos-project/pylot/blob/master/pylot/control/utils.py)**

a module that contains some utility functions for the control package, such as converting between different coordinate systems, computing the distance and angle between two points, and clipping the control inputs to the limits1

## MPC Control Code

The MPC Operator is a component of the Pylot Carla project that uses model predictive control to drive the vehicle. It works by solving an optimization problem that minimizes the **deviation from the desired waypoints** and **speed limit**, while **respecting the physical constraints** of the vehicle.
The MPC Operator has the following steps:

- Receive the waypoints and speed limit from the Planning Operator. The waypoints are a list of coordinates that define the path to follow, and the speed limit is the maximum allowed speed on the road.
- Convert the waypoints from the world coordinate system to the vehicle coordinate system. This makes it easier to compute the error and the control inputs.
- Set up the kinematic model of the vehicle, which describes how the state of the vehicle (position, velocity, heading, etc.) changes with respect to the control inputs (throttle, steering, etc.).
- Define the constraints of the optimization problem, which include the bounds on the control inputs and the state variables, as well as the initial and final conditions.
- Define the cost function of the optimization problem, which is a weighted sum of the errors between the predicted and desired states. The cost function can be tuned by changing the weights of the different terms, such as the cross-track error, the heading error, the speed error, etc.
- Solve the optimization problem using the **`cvxpy`** library, which uses a convex solver to find the optimal control inputs that minimize the cost function.
- Send the control commands to the Simulator Operator or the Car Operator, which apply the throttle, steering, and braking to the vehicle.

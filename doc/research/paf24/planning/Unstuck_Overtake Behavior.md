# Unstuck/Overtake Behavio

**Summary:** This document analyze current unstuck/overtake behavior and reveal several critical issues with the current implementation 
- [Unstuck behavior](#unstuck-behavior)
    -[Key components](#key-components)
    -[How unstuck is triggered](#how-unstuck-is-triggered)
-[Overtake behavior](#overtake-behavior)
    -[Key components](#key-components) 
    -[How overtake is triggered](#how-overtake-is-triggered)
- [Relevant rostopics](#relevant-rostopics)
- [Methods overview](#methods-overview)
    -[def __set_curr_behavior](#__set_curr_behavior)
    -[def change_trajectory](#change_trajectory)
    -[def overtake_fallback](#overtake_fallback)
- [Overtake behavior issues](#overtake-behavior-issues)
    -[Aggressive lane changes](#aggressive-lane-changes)
    -[Inadequate collision detection](#inadequate-collision-detection)
    -[Improper trajectory planning](#improper-trajectory-planning)
- [Unstuck behavior issues](#unstuck-behavior-issues)
- [Potential improvements](#potential-improvements)



## Unstuck behavior:
    The "unstuck" behavior is designed to address situations where the vehicle becomes stuck, perhaps due to obstacles or other vehicles blocking its path.
    ### Key components:
        self.unstuck_distance: Tracks the distance to the object causing the blockage, helping the algorithm to determine if an alternative route (overtaking) might be required.
        self.unstuck_overtake_flag: Prevents repeated or "spam" overtakes by ensuring that the unstuck overtake only happens once within a specified distance (UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE).
        self.init_overtake_pos: Saves the position where the unstuck behavior is initiated to prevent excessive overtaking within a short range.
        Overtaking Logic: The method overtake_fallback is called within __get_speed_unstuck, where a new trajectory is created by offsetting the path to one side, allowing the vehicle to bypass the obstacle and get "unstuck."


    ### How unstuck is triggered:
        The method __get_speed_unstuck checks the current behavior (e.g., us_unstuck), adjusting speed accordingly and invoking overtake_fallback if an obstacle is detected and the vehicle is in an "unstuck" situation.
        UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE ensures that once an unstuck maneuver is initiated, it won't be repeated until a certain distance is cleared

## Overtake behavior:
    The overtake behavior allows the vehicle to safely overtake a slower-moving or stationary vehicle ahead by modifying its trajectory.
    ### Key components:
        self.__overtake_status: Manages the status of overtaking, where 1 indicates an active overtake, while -1 signals that overtaking isn't required.
        change_trajectory: Initiates the overtake by calling overtake_fallback when it detects a collision point or slow-moving vehicle within a certain range. This method also publishes the updated __overtake_status to signal other systems of the change.
        overtake_fallback: Calculates a new trajectory path that offsets the vehicle by a certain distance (normal_x_offset) to safely pass the obstacle. The adjusted path uses the Rotation library to align the offset with the vehicle’s heading, creating a smooth path around the obstacle.

    ### How overtake is triggered:
        The method __set_curr_behavior monitors the vehicle’s behavior, triggering an overtake when ot_enter_init behavior is detected and if an obstacle or collision point is near. It then calls change_trajectory to modify the route.
        Speed adjustments specific to overtaking are handled in __get_speed_overtake, where the vehicle might slow down (e.g., ot_enter_slow) or proceed at normal speed after overtaking (e.g., ot_leave).
       
## Relevant rostopics   
   overtake_success 
   unstuck_distance
   ...
   #TODO:need to continue



## Methods overview
    ## How the function def __set_curr_behavior(self, data: String) works:

    If beggins overtake manuever:
        if no obsticle to overtake:
                unsuccessfull or canceled overtake
                publish abandonment of the overtake attempt.
        else:
        change trajectory
        
    Infinity as "No Collision Detected": Setting self.__collision_point to infinity (np.inf) would mean "no collision detected." So, checking if np.isinf(self.__collision_point) effectively asks, "Is there no obstacle or point that needs an overtake?" (??? not sure about it)
    self.__overtake_status = -1 #unsuccessfull or canceled overtake
    self.__overtake_status = 1 #overtake should take place

    **Questions:**
        1. who tells the function the behaviour status?
        2. If the function has already become an overtake status, why should we check if there is actually an object to overtake?

    ## How the function change_trajectory(self, distynce_obj) works:
    update trajectory for overtaking and convert it to a new Path message. Args:distance_obj (float): distance to overtake object
    Publish overate success status

    **Questions:** 
        1. What does the success status for overtake mean? It seems like it oublishs than an overtake takes place but does not notify that the overtake manoeuvre was successful. 

    ## How the function def overtake_fallback(self, distance, pose_list, unstuck=False) works: 
    This method constructs a temporary path around an obstacle based on the current location, intended distance to overtake, and a choice between two different lateral offsets depending on whether the vehicle is in an “unstuck” situation.

    **Pick Path Points Around the Obstacle:** It selects a section of the path around the vehicle’s current position and the obstacle it needs to overtake. If the vehicle is stuck, it picks a larger section to give it more room to maneuver.
    **Shift Path to the Side:** It moves this section of the path slightly to the side (left or right, depending on the vehicle's heading) to avoid the obstacle. If the vehicle is in a “stuck” situation, it shifts it a bit more to give it extra clearance.
    **Create the New Path:** It converts the shifted points into a path format that the vehicle’s navigation system can understand and follow.
    **Combine with Original Path:** it merges this temporary bypass with the original path, so the vehicle can return to its route after it passes the obstacle.

## Overtake behavior issues
    ### Aggressive lane changes: The vehicle exhibits aggressive lane changes, leading to emergency stops to avoid oncoming traffic. This behavior suggests that the decision-making logic for overtaking does not adequately assess the surrounding traffic conditions before executing maneuvers.

    ### Inadequate collision detection: The vehicle fails to avoid obstacles, such as open car doors or stationary vehicles, indicating that the collision detection mechanisms are either insufficient or not effectively integrated into the overtaking logic.

    ### Improper trajectory planning: The trajectory generated for overtaking often leads to incorrect paths, such as attempting to overtake trees or parked cars without considering oncoming traffic. A better filtering and validation of potential overtaking targets is needed.

## Unstuck behavior issues
    The vehicle gets stuck multiple times and exhibits erratic behavior when trying to get unstuck,
    The vehicle's aggressive lane-holding behavior after being unstuck
    After collisions, the vehicle often fails to resume movement, indicating a lack of recovery logic post-collision

    
## Potential improvements
    'def change trajektory' : consider implementing a more sophisticated trajectory planning algorithm that takes into account dynamic obstacles and traffic conditions rather than relying on a fallback method. Verify that the area into which the vehicle is moving is clear of obstacles and oncoming traffic.
    'def overtake_fallback' :  instead of fixed offsets for unstuck and normal situations, consider dynamically calculating offsets based on current speed, vehicle dimensions, and surrounding traffic conditions.
    '__get_speed_unstuck' : includ checks for nearby vehicles and their speeds. Make UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE dynamic.
    '__check_emergency' : Now it only considers whether the vehicle is in a parking behavior state. Expand this method to evaluate various emergency conditions (e.g., obstacles detected by sensors) and initiate appropriate responses (e.g., stopping or rerouting).
    'get_speed_by_behavior' : consider feedback from sensors regarding current traffic conditions.
    '__calc_corner_points' : thinking about somathing smarter rather than a simple angle thresholds.





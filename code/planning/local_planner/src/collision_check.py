#!/usr/bin/env python
# import rospy
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from carla_msgs.msg import CarlaRoute   # , CarlaWorldInfo
# from nav_msgs.msg import Path
# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray


class CollisionCheck(CompatibleNode):
    """
    This is currently a test node. In the future this node will be
    responsible for detecting collisions and reporting them.
    """

    def __init__(self):
        super(CollisionCheck, self).__init__('CollisionCheck')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.current_speed = 50 / 3.6  # m/ss
        # TODO: Add Subscriber for Speed and Obstacles
        self.logdebug("CollisionCheck started")

    def update(self, speed):
        self.current_speed = speed

    def time_to_collision(self, obstacle_speed, distance):
        return distance / (self.current_speed - obstacle_speed)

    def meters_to_collision(self, obstacle_speed, distance):
        return self.time_to_collision(obstacle_speed, distance) * \
            self.current_speed

    # PAF 22
    def calculate_safe_dist(self) -> float:
        """
        Calculates the distance you have to keep to the vehicle in front to
        have t_reaction to react to the vehicle suddenly stopping
        The formula replicates official recommendations for safe distances
        """
        t_reaction = 1  # s
        t_breaking = 1  # s
        a = 8  # m/s^2
        v = self.current_speed
        s = - 0.5 * a * t_breaking ** 2 + v * t_breaking + v * t_reaction
        return s + 5

    def calculate_rule_of_thumb(self, emergency):
        reaction_distance = self.current_speed
        braking_distance = (self.current_speed * 0.36)**2
        if emergency:
            return reaction_distance + braking_distance / 2
        else:
            return reaction_distance + braking_distance

    def check_crash(self, obstacle):
        distance, obstacle_speed = obstacle

        collision_time = self.time_to_collision(obstacle_speed, distance)
        collision_meter = self.meters_to_collision(obstacle_speed, distance)

        safe_distance = self.calculate_safe_dist()
        safe_distance2 = self.calculate_rule_of_thumb(False)
        emergency_distance2 = self.calculate_rule_of_thumb(True)

        # TODO: Convert to Publishers
        if collision_time > 0:
            if distance < emergency_distance2:
                print(f"Emergency Brake needed, {emergency_distance2:.2f}")
            print(f"Ego reaches obstacle after {collision_time:.2f} seconds.")
            print(f"Ego reaches obstacle after {collision_meter:.2f} meters.")
            print(f"Safe Distance PAF 22: {safe_distance:.2f}")
            print(f"Safe Distance Thumb: {safe_distance2:.2f}")
        else:
            print("Ego slower then car in front")

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            pass

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the CollisionCheck node
    :param args:
    """
    roscomp.init('CollisionCheck')

    try:
        node = CollisionCheck()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

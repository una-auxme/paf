#!/usr/bin/env python
import rospy
import numpy as np
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber, Publisher
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaSpeedometer   # , CarlaWorldInfo
from nav_msgs.msg import Path
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, Float32


class ACC(CompatibleNode):
    """
    This node recieves a possible collision and 
    """

    def __init__(self):
        super(ACC, self).__init__('ACC')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.current_speed = 50 / 3.6  # m/ss
        
        self.logdebug("ACC started")
        # TODO: Add Subscriber for Obsdacle from Collision Check
        # self.obstacle_sub: Subscriber = self.new_subscription(
        # )

        # Get current speed
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        # Get initial set of speed limits
        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1)

        # Get trajectory to determine current speed limit
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        # Get current position to determine current speed limit
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        # Publish desiored speed to acting
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/acc_velocity",
            qos_profile=1)

        # List of all speed limits, sorted by waypoint index
        self.__speed_limits_OD: [float] = []
        # Current Trajectory
        self.__trajectory: Path = None
        # Current index from waypoint
        self.__current_wp_index: int = 0
        # Current speed
        self.__current_velocity: float = None
        # Is an obstacle ahead where we would collide with?
        self.collision_ahead: bool = False
        # Distnace and speed from possible collsion object
        self.__obstacle: tuple = None
        # Current speed limit
        self.speed_limit: float = np.Inf
        

    def calculate_safe_speed(self, distance, object_speed, own_speed):
        """calculates the speed to meet the desired distance to the object

        Args:
            distance (float): Distance to the object in front
            object_speed (float): Speed from object in front
            own_speed (float): Current speed of the ego vehicle

        Returns:
            float: safe speed tp meet the desired distance
        """
        safety_distance = own_speed/2  
        if distance < safety_distance:
            # If safety distance is reached, we want to reduce the speed to meet the desired distance
            # The speed is reduced by the factor of the distance to the safety distance
            # Another solution could be object_speed - (safety_distance-distance)

            safe_speed = object_speed * (distance / safety_distance)
            return safe_speed
        else:
            # If safety distance is reached, drive with same speed as Object in front
            # TODO: Incooperate overtaking -> Communicate with decision tree about overtaking
            return object_speed

    def __get_current_velocity(self, data: CarlaSpeedometer):
        """_summary_

        Args:
            data (CarlaSpeedometer): _description_
        """
        self.__current_velocity = float(data.speed)
        self.velocity_pub.publish(self.__current_velocity)

    def __set_trajectory(self, data: Path):
        self.__trajectory = data

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        self.__speed_limits_OD = data.data

    def __current_position_callback(self, data: PoseStamped):
        if len(self.__speed_limits_OD) < 1 or self.__trajectory is None:
            return

        agent = data.pose.position
        current_wp = self.__trajectory.poses[self.__current_wp_index].\
            pose.position
        next_wp = self.__trajectory.poses[self.__current_wp_index + 1].\
            pose.position

        # distances from agent to current and next waypoint
        d_old = abs(agent.x - current_wp.x) + abs(agent.y - current_wp.y)
        d_new = abs(agent.x - next_wp.x) + abs(agent.y - next_wp.y)
        if d_new < d_old:
            # update current waypoint and corresponding speed limit
            self.__current_wp_index += 1
            self.speed_limit = \
                self.__speed_limits_OD[self.__current_wp_index]

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            if self.__velocity is None:
                self.logdebug("ACC hasn't received the velocity of the ego "
                              "vehicle yet and can therefore not publish a "
                              "velocity")
                return
            
            # check if collision is ahead 
            if self.collision_ahead:
                # collision is ahead
                # check if object moves
                if self.obstacle_speed > 0:
                    # Object is moving
                    # caluculate safe speed
                    speed = self.calculate_safe_speed()
                    self.velocity_pub.publish(speed)
                else:
                    # If object doesnt move, behaviour tree will handle overtaking or emergency stop was done by collision check
                    pass
            else:
                # no collisoion ahead -> publish speed limit
                self.velocity_pub.publish(self.speed_limit)
        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the ACC node
    :param args:
    """
    # roscomp.init('ACC')

    # try:
    #     node = ACC()
    #     node.run()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     roscomp.shutdown()

    print("ACC")
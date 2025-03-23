import pygame
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


from typing import List, Tuple


ORIGIN_X = 984.5
ORIGIN_Y = -5442.0


class Trajectory:
    z_visual: float = 0

    def __init__(self, start_x: float, start_y: float, screen_factor: float):
        self.screen_factor = screen_factor
        self.start_x = start_x
        self.start_y = start_y

        self.current_trajectory: List[Tuple[float, float]] = []
        self.trajectory_sub = rospy.Subscriber(
            name="/paf/planning/trajectory",
            data_class=Path,
            callback=self.set_trajectory,
        )

    def set_trajectory(self, path_msg: Path):
        trajectory: List[Tuple[float, float]] = []
        pose_stamped: PoseStamped
        for pose_stamped in path_msg.poses:
            trajectory.append(
                (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            )

        self.current_trajectory = [(x, y) for x, y in trajectory]

    def draw(self, screen):
        # Most important is to swap to y, x for drawing
        screen_trajectory = [
            (y * self.screen_factor, x * self.screen_factor)
            for x, y in self.current_trajectory
        ]
        if len(screen_trajectory) > 2:
            pygame.draw.lines(screen, (0, 255, 255), False, screen_trajectory, 2)

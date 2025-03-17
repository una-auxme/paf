#!/usr/bin/env python

import rospy
import random
import math

import pygame
from pygame.locals import *

from sim.msg import (
    VehicleCtrl,
    VehicleInfo,
    Entity,
    Entities,
    Vector2D,
    VisPath,
    MultiPath,
)

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaSpeedometer

from sim.car import ScreenCar
from sim.collider import ScreenCollider
from sim.button import Button
from sim.trajectory import Trajectory

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from typing import Tuple, List, Optional

speed = 5

window_width = 800
window_height = 600

screen_factor: float = 8


target_velocity = 5


class Simulation:
    paused: bool = False

    target: ScreenCollider = ScreenCollider(30.0, 30.0, screen_factor, (0, 0, 255))
    objects: List[ScreenCollider] = []

    def __init__(self):
        rospy.init_node("car_simulation", anonymous=True)
        pygame.display.set_caption("Car Simulation with Object Placement")
        self.screen = pygame.display.set_mode(
            (window_width, window_height), flags=pygame.DOUBLEBUF
        )
        self.reset_button = Button(40, 40, "Reset")
        self.pause_button = Button(150, 40, "Pause")
        self.clear_button = Button(230, 40, "Clear")

        self.steer: float = 0.0
        self.throttle: float = 1.0
        self.brake: float = 0.0

        car_start_x: float = rospy.get_param("car_start_x", 30)
        car_start_y: float = rospy.get_param("car_start_y", 100)
        car_start_theta: float = -math.pi / 2
        self.car = ScreenCar(car_start_x, car_start_y, car_start_theta, screen_factor)

        self.multi_path: Optional[MultiPath] = None

        rospy.Subscriber(
            "/carla/hero/vehicle_control_cmd",
            CarlaEgoVehicleControl,
            self.ctrl_callback,
        )
        rospy.Subscriber("multi_path", MultiPath, self.multi_path_callback)
        self.pos_pub = rospy.Publisher("info", VehicleInfo, queue_size=10)

        self.speedometer_pub = rospy.Publisher(
            "carla/hero/Speed", CarlaSpeedometer, queue_size=10
        )
        self.target_velocity_pub = rospy.Publisher(
            "paf/hero/target_velocity", Float32, queue_size=10
        )

        self.current_pos_pub = rospy.Publisher(
            "paf/hero/current_pos", PoseStamped, queue_size=10
        )

        self.current_heading_pub = rospy.Publisher(
            "paf/hero/current_heading", Float32, queue_size=10
        )

        self.entities_pub = rospy.Publisher("entities", Entities, queue_size=10)
        self.target_pub = rospy.Publisher("target", Vector2D, queue_size=10)

        self.trajectory = Trajectory(car_start_x, car_start_y, screen_factor)

        rate_hz = 20
        duration = rospy.Duration(1.0 / rate_hz)
        self.dt = duration.to_sec()
        self.rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

        # rospy.timer.Timer(duration, lambda _: self.update())
        """We would love to use this timer here. And the timer does work if not in docker container. 
        Inside the container the timer somehow works differently and we cannot use it.
        """

    def update(self):
        self._publish_position()
        self._publish_entities()
        self._publish_target()
        self._run_simulation()

    def ctrl_callback(self, msg: CarlaEgoVehicleControl):
        self.steer = msg.steer
        self.throttle = msg.throttle
        self.brake = msg.brake

    def multi_path_callback(self, msg: MultiPath):
        self.multi_path = msg

    def _publish_target(self):
        msg = Vector2D()
        msg.x, msg.y = self.target.get_position()
        self.target_pub.publish(msg)

    def _publish_position(self):
        msg = VehicleInfo()
        msg.x, msg.y = self.car.get_position()
        msg.theta = self.car.get_phi()
        msg.velocity = self.car.v

        self.pos_pub.publish(msg)

        msg = CarlaSpeedometer()
        msg.speed = self.car.v
        self.speedometer_pub.publish(msg)

        msg = Float32()
        msg.data = target_velocity
        self.target_velocity_pub.publish(msg)

        msg = Float32()
        msg.data = self.car.get_phi()
        self.current_heading_pub.publish(msg)

        msg = PoseStamped()
        msg.pose.position.x, msg.pose.position.y = self.car.get_position()
        self.current_pos_pub.publish(msg)

    def _publish_entities(self):
        """Publishes entities in car coordinates"""
        msg = Entities()
        car_x, car_y = self.car.get_position()
        car_phi = self.car.get_phi()
        for obj in self.objects:
            ent = Entity()
            ent.x, ent.y = obj.to_other_coordinates(car_x, car_y, car_phi)
            ent.radius = obj.radius
            msg.entities.append(ent)
        self.entities_pub.publish(msg)

    def _draw_objects(self):
        for obj in self.objects:
            obj.draw(self.screen)

    def _draw_multipath(self):
        if self.multi_path is None:
            return

        for path in self.multi_path.paths:
            path: VisPath
            points = [
                [int(p.y * screen_factor), int(p.x * screen_factor)]
                for p in path.points
            ]
            pygame.draw.lines(self.screen, (120, 120, 0), False, points, 2)

        self.multi_path = None
        #! Transform left to right

    def _draw_vehicle_info(self):
        v = self.car.v
        theta = self.car.theta
        label: str = (
            f"Vel:     {round(v,2)} m/s",
            f"Theta:   {round(theta,2)} rad",
            f"Steer:   {round(self.steer,2)}",
            f"Throttle:{round(self.throttle,2)}",
            f"Brake:   {round(self.brake,2)}",
        )
        y_offset = 0
        for line in label:
            text = pygame.font.Font(None, 18).render(line, True, (0, 0, 0))
            self.screen.blit(
                text,
                (
                    window_width - 200,
                    window_height - 100 + y_offset,
                ),
            )
            y_offset += 18

    def _run_simulation(self):
        if not self.paused:
            self.car.update(
                steering=-self.steer,
                throttle=self.throttle,
                brake=self.brake,
                dt=self.dt,
            )

        for event in pygame.event.get():
            if event.type == QUIT:
                rospy.signal_shutdown("User quit the window.")
                pygame.quit()
                return

            # Place objects on mouse click
            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    if self.reset_button.check_collision(mouse_x, mouse_y):
                        self._reset()
                    elif self.pause_button.check_collision(mouse_x, mouse_y):
                        self.paused = not self.paused
                    elif self.clear_button.check_collision(mouse_x, mouse_y):
                        self.objects = []
                    else:
                        # Randomly color the object
                        color = (
                            random.randint(0, 255),
                            random.randint(0, 255),
                            random.randint(0, 255),
                        )
                        obj = ScreenCollider(0, 0, screen_factor, color)
                        obj.set_position_screen(mouse_x, mouse_y)
                        self.objects.append(obj)
                if event.button == 3:  # RIGHT CLICK
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    self.target.set_position_screen(mouse_x, mouse_y)

        # Clear screen before drawing the car and objects
        self.screen.fill((255, 255, 255))  # White background
        self.car.draw_grid(self.screen)

        self.car.draw(self.screen)
        self.car.draw_history(self.screen)
        self._draw_objects()
        self.pause_button.draw(self.screen)
        self.reset_button.draw(self.screen)
        self.clear_button.draw(self.screen)
        self._draw_multipath()
        # self.target.draw(self.screen)
        self.trajectory.draw(self.screen)

        self._draw_vehicle_info()
        # Update display
        pygame.display.update()

    def _reset(self):
        self.car.reset()


if __name__ == "__main__":
    pygame.init()
    try:
        sim = Simulation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

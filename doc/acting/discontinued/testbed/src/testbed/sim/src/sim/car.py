from typing import Tuple, List, Optional
import math
import pygame


class Car:
    wheelbase: float = 1.8  # Wheelbase
    length: float = 4.5

    max_steering_angle: float = 70  # Steering angle in deg
    max_velocity: float = 20  # maximum velocity
    min_velocity: float = 0  # for now 0

    _start_position: Tuple[float, float] = (0, 0)
    _start_phi: float = 0.0

    def __init__(self, x: float = 0, y: float = 0, phi: float = 0):
        self._start_position = (x, y)
        self._start_phi = phi
        self.reset()

    def reset(self):
        self.x, self.y = self._start_position
        self.theta = self._start_phi
        self.v = 0

        self.steering_angle: float = 0.0

    def update(self, steering: float, throttle: float, brake: float, dt: float) -> None:
        self.steering_angle = math.radians(
            steering * self.max_steering_angle
        )  # max steering angle
        if throttle > 0:
            self.v += dt * throttle
        if brake > 0:
            self.v -= dt * brake
        # add some drag
        k = 0.003
        self.v -= dt * k * self.v**2

        self.v = max(self.v, self.min_velocity)
        self.v = min(self.v, self.max_velocity)

        self._update(dt)

    def _update(self, dt):
        """
        Update the car's state.
        :param delta: Steering angle (in radians)
        :param velocity: Target velocity (m/s)
        :param brake: Brake force (0-1, where 1 is full brake)
        :param dt: Time step for the update (seconds)
        """
        # Update the car's position and orientation based on the Ackermann model
        if self.v != 0:
            if self.steering_angle != 0:
                # Turning radius based on steering angle and wheelbase
                turning_radius = self.wheelbase / math.tan(self.steering_angle)
                # Calculate the change in orientation (theta) based on velocity and turning radius
                delta_theta = self.v / turning_radius * dt

                # Update the orientation (heading) of the car
                self.theta += delta_theta
            # Normalize theta to be between -pi and pi
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

            # Update the position (x, y) based on the current velocity and orientation
            self.x += self.v * math.cos(self.theta) * dt
            self.y += self.v * math.sin(self.theta) * dt
        else:
            # If velocity is zero, no movement occurs
            pass

    def get_position(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def get_phi(self) -> float:
        return self.theta


class ScreenCar(Car):
    color: Tuple[float, float, float] = (0, 255, 0)
    history_color: Tuple[float, float, float] = (255, 0, 0)
    grid_color: Tuple[float, float, float] = (250, 250, 250)

    def __init__(self, x: float, y: float, phi: float, screen_factor: float):
        super().__init__(x, y, phi)
        self._screen_factor = screen_factor
        self._size: Tuple[float, float] = (
            self.wheelbase * screen_factor,
            self.length * screen_factor,
        )

    def update(self, steering: float, throttle: float, brake: float, dt: float) -> None:
        super().update(steering, throttle, brake, dt)
        self.history.append(self._position_to_screen())

    def reset(self):
        super().reset()
        self.history: List[Tuple[float, float]] = []

    def draw(self, screen):
        points = []
        x, y = self._position_to_screen()
        height, width = self._size
        rotation = -self.theta

        # The distance from the center of the rectangle to
        # one of the corners is the same for each corner.
        radius = math.sqrt((height / 2) ** 2 + (width / 2) ** 2)

        # Get the angle to one of the corners with respect
        # to the x-axis.
        angle = math.atan2(height / 2, width / 2)

        # Transform that angle to reach each corner of the rectangle.
        angles = [angle, -angle + math.pi, angle + math.pi, -angle]

        # Calculate the coordinates of each point.
        for angle in angles:
            y_offset = -1 * radius * math.sin(angle + rotation)
            x_offset = radius * math.cos(angle + rotation)

            # Lastly switch x and y for right handed
            points.append((int(y + y_offset), int(x + x_offset)))

        pygame.draw.polygon(screen, self.color, points)

    def draw_history(self, screen):
        if len(self.history) > 1:
            # !! Swap right handed data to left handed draw
            pygame.draw.lines(
                screen, self.history_color, False, [(y, x) for x, y in self.history], 2
            )

    def draw_grid(self, screen):
        width, height = self._width_height_from_screen(screen)
        for x in range(0, width, self._screen_factor):
            pygame.draw.line(screen, self.grid_color, (x, 0), (x, height))

        # Draw horizontal lines
        for y in range(0, height, self._screen_factor):
            pygame.draw.line(screen, self.grid_color, (0, y), (width, y))

    def _width_height_from_screen(self, screen) -> Tuple[int, int]:
        _, __, width, height = screen.get_rect()
        return (width, height)

    def _position_to_screen(self) -> Tuple[int, int]:
        return (int(self.x * self._screen_factor), int(self.y * self._screen_factor))

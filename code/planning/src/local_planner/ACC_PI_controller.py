#!/usr/bin/env python
from simple_pid import PID


class ACC_PI_Controller:
    def __init__(self, Kp, Ki, T_gap):
        self.pid = PID(Kp, Ki, 0)
        self.T_gap = T_gap

    def update(self, d, v_self, v_lead):
        """
        Update the desired speed based on the distance (d) to the leading car,
        the self speed (v_self), and the leading car's speed (v_lead).
        """
        # Calculate the desired gap based on a time gap model
        d_desired = self.T_gap * v_self

        # Calculate the error in distance (gap)
        error = abs(d_desired - d)

        # Use the PI controller to compute the control signal (adjustment in speed)
        speed_adjustment = self.pid(error)

        # Calculate the new speed (this might involve limiting speed for safety)
        new_speed = v_self + speed_adjustment

        # Ensure that the car's speed doesn't exceed the speed of the leading car
        # new_speed = min(new_speed, v_lead)

        return new_speed, error


"""def simulate_ACC_system():
    # Define PI controller parameters
    Kp = 1.0  # Proportional gain
    Ki = 0.0  # Integral gain
    T_gap = 2.0  # Desired time gap in seconds

    # Initialize the PI controller
    acc_controller = ACC_PI_Controller(Kp, Ki, T_gap)

    # Example initial values
    v_self = 20  # Your car's current speed in m/s (72 km/h)
    v_lead = 15  # Speed of the leading car in m/s (54 km/h)
    d = 30  # Current distance to the leading car in meters

    # Simulate updating the speed in a loop
    for _ in range(100):  # Simulate 10 steps
        new_speed, error = acc_controller.update(d, v_self, v_lead)
        print(f"Desired Speed: {new_speed:.2f} m/s, Error: {error:.2f} m")

        # Update your car's speed and distance to the leading car (this would be a model in a real system)
        v_self = new_speed  # Update the car's speed
        d = max(0, d - (v_self - v_lead))  # Update distance (simplified)


simulate_ACC_system()"""

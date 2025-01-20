#!/usr/bin/env python
from simple_pid import PID
import time
import matplotlib.pyplot as plt

# speedlimit in city: 11.11 m/s = 40 km/h


class ACC_PI_Controller:
    def __init__(self, Kp, Ki, T_gap, speed_limit):
        self.pid = PID(Kp, Ki, 0)
        self.T_gap = T_gap
        self.pid.output_limits = (-10, speed_limit)

    def update(self, d, v_self, v_lead, speed_limit):
        """
        Update the desired speed based on the distance (d) to the leading car,
        the self speed (v_self), and the leading car's speed (v_lead).
        """
        # Calculate the desired gap based on a time gap model
        d_desired = self.T_gap * v_self

        # Calculate the error in distance (gap)
        error = d_desired - d

        # Use the PI controller to compute the control signal (adjustment in speed)
        speed_adjustment = self.pid(error)

        # Calculate the new speed (this might involve limiting speed for safety)
        new_speed = v_self + speed_adjustment

        # if new_speed > speed_limit:
        #    new_speed = speed_limit

        # Ensure that the car's speed doesn't exceed the speed of the leading car
        # new_speed = min(new_speed, v_lead)

        return new_speed, error


def simulate_ACC_system():

    # Define PI controller parameters
    # Kp = 0.2  # Proportional gain
    Ki = 0.0  # Integral gain
    T_gap = 2.0  # Desired time gap in seconds
    speed_limit = 11.11

    fig, axes = plt.subplots(3, 4, figsize=(12, 9))
    i = 0
    j = 0

    # Example initial values
    v_self = 10  # car's current speed in m/s
    v_lead = 20  # Speed of the leading car in m/s
    d = 10  # Current distance to the leading car in meters
    delta_t = 0.05  # control loop rate in planning.launch

    for Kp in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2]:
        # Initialize the PI controller
        acc_controller = ACC_PI_Controller(Kp, Ki, T_gap, speed_limit)
        # Simulate updating the speed in a loop
        d_list = [d]
        t_list = [0.0]

        for _ in range(500):  # Simulate 10 steps
            new_speed, error = acc_controller.update(d, v_self, v_lead, speed_limit)
            print(f"Desired Speed: {new_speed:.2f} m/s, Error: {error:.2f} m")

            # Update your car's speed and distance to the leading car (this would be
            # a model in a real system)
            v_self = new_speed  # Update the car's speed
            delta_d_self = v_self * delta_t
            delta_d_lead = v_lead * delta_t
            d = d - delta_d_self + delta_d_lead
            d_list.append(d)
            t_list.append(t_list[-1] + delta_t)
            # d = max(0, d - (v_self - v_lead))  # Update distance (simplified)
            time.sleep(delta_t)  # wait 10 ms

        ax = axes[i, j]
        ax.plot(t_list, d_list)
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Distance (meters)")
        ax.set_title(f"Distance vs Time for Kp = {Kp}")
        j += 1
        if j > 3:
            j = 0
            i += 1

    fig.suptitle(
        f"Simulation for v_lead = {v_lead}, v_self = {v_self}, d = {d}", fontsize=16
    )
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    # Show the plot
    plt.show()


simulate_ACC_system()

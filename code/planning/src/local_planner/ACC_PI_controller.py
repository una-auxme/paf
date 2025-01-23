#!/usr/bin/env python
from simple_pid import PID
import time
import matplotlib.pyplot as plt

# speedlimit in city: 11.11 m/s = 40 km/h


class ACC_PI_Controller:
    def __init__(self, Kp, Ki, T_gap, speed_limit):

        # self.pid = PID(Kp, Ki, 0)
        # self.pid.output_limits = (-3, 3)
        self.T_gap = T_gap
        self.d_min = 2
        self.k_i = Ki
        self.k_p = Kp

    def update(self, d_current, v_self, v_lead, speed_limit):
        """
        Update the desired speed based on the distance (d) to the leading car,
        the self speed (v_self), and the leading car's speed (v_lead).
        """

        d_desired = self.d_min + self.T_gap * v_self

        v_desired = (
            self.k_i * (d_current - d_desired) + self.k_p * (v_lead - v_self) + v_self
        )

        return v_desired


def simulate_ACC_system():

    # Define PI controller parameters
    # Kp = 0.2  # Proportional gain
    Ki = 0.0  # Integral gain
    T_gap = 2.0  # Desired time gap in seconds
    speed_limit = 11.11

    fig, axes = plt.subplots(3, 4, figsize=(12, 9))
    i = 0
    j = 0

    v_self_init = 8
    v_lead_init = 11.1
    d_init = 50
    delta_t = 0.05

    for Kp in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2]:
        # Initialize the PI controller
        acc_controller = ACC_PI_Controller(Kp, Ki, T_gap, speed_limit)
        # Simulate updating the speed in a loop

        # Example initial values
        v_self = v_self_init  # car's current speed in m/s
        v_lead = v_lead_init  # Speed of the leading car in m/s
        d = d_init  # Current distance to the leading car in meters

        d_list = [d]
        t_list = [0.0]

        for _ in range(500):  # Simulate x steps
            new_speed = acc_controller.update(d, v_self, v_lead, speed_limit)
            print(f"Distance: {d}, Desired Speed: {new_speed:.2f} m/s")

            # Update the car's speed and distance to the leading car
            v_self = new_speed  # Update the car's speed
            delta_d_self = v_self * delta_t
            delta_d_lead = v_lead * delta_t
            d = d - delta_d_self + delta_d_lead
            d_list.append(d)
            t_list.append(t_list[-1] + delta_t)
            # d = max(0, d - (v_self - v_lead))  # Update distance (simplified)
            time.sleep(delta_t)  # wait 10 ms
            # v_self = new_speed

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
        f"Simulation for v_lead = {v_lead_init}, v_self = {v_self_init}, d = {d_init}",
        fontsize=16,
    )
    plt.tight_layout()
    plt.subplots_adjust(top=0.9)
    # Show the plot
    plt.show()


simulate_ACC_system()

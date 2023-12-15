
from collections import namedtuple


def convert_to_ms(speed):
    return speed / 3.6


Behavior = namedtuple("Behavior", ("name", "speed"))

# Changed target_speed_pub to curr_behavior_pub

# Intersection - Behaviors

# Approach

int_app_init = Behavior("int_app_init", convert_to_ms(30.0))

# No Traffic Light or Sign -> stop dynamically at Stopline
int_app_no_sign = Behavior("int_app_no_sign", -2)

int_app_green = Behavior("int_app_green", convert_to_ms(30.0))

# Wait

int_wait = Behavior("int_wait", 0)

# Enter

int_enter_no_light = Behavior("int_enter_no_light", convert_to_ms(50.0))

int_enter_empty_str = Behavior("int_enter_empty_string", convert_to_ms(18.0))

int_enter_light = Behavior("int_enter_light", convert_to_ms(50.0))

# Exit

int_exit = Behavior("int_exit", -1)  # Get SpeedLimit dynamically


# Lane Change

# Approach

lc_app_init = Behavior("lc_app_blocked", convert_to_ms(30.0))


# TODO: Find out purpose of v_stop in lane_change (lines: 105 - 128)
lc_app_blocked = Behavior("lc_app_blocked", 0.5)

# Wait

# Has a publisher but doesnt publish anything ??

# Enter

lc_enter_init = Behavior("lc_enter_init", convert_to_ms(20.0))

# Exit

lc_exit = Behavior("lc_exit", -1)  # Get SpeedLimit dynamically


# Cruise

cruise = Behavior("Cruise", -1)

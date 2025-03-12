"""
This file contains the behavior names.

Before PR #740, the names were also coupled with fixed speed values
connected to the behaviors. As they are completely unused in favor
of the ACC since PR #725, the speeds were remove from this file.
"""


class Behavior:
    def __init__(self, name):
        self.name = name


# Leave Parking space
parking = Behavior("parking")

# Intersection
# Approach
int_app_init = Behavior("int_app_init")
int_app_to_stop = Behavior("int_app_to_stop")
int_app_green = Behavior("int_app_green")
# Wait
int_wait = Behavior("int_wait")
# Enter
int_enter = Behavior("int_enter")

# Lane Change
# Approach
lc_app_init = Behavior("lc_app_init")
lc_app_blocked = Behavior("lc_app_blocked")
lc_app_free = Behavior("lc_app_free")
# Wait
lc_wait = Behavior("lc_wait")
lc_wait_free = Behavior("lc_wait_free")
# Enter
lc_enter_init = Behavior("lc_enter_init")
# Exit
lc_exit = Behavior("lc_exit")

# Overtake
# Approach
ot_app_blocked = Behavior("ot_app_blocked")
ot_app_free = Behavior("ot_app_free")
# Wait
ot_wait = Behavior("ot_wait")
ot_wait_free = Behavior("ot_wait_free")
# Enter
ot_enter_init = Behavior("ot_enter_init")
# Exit
ot_leave = Behavior("ot_leave")

# Cruise
cruise = Behavior("cruise")

# Unstuck Routine
us_unstuck = Behavior("us_unstuck")
us_forward = Behavior("us_forward")
us_stop = Behavior("us_stop")

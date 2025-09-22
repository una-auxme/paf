from typing import Optional
import py_trees

from rclpy.client import Client

from planning_interfaces.srv import SpeedAlteration


SPEED_OVERRIDE_ID: str = "/speed/override"
"""Blackboard: Contains a float of the desired speed of the vehicle
"""
SPEED_LIMIT_ID: str = "/speed/limit"
"""Blackboard: Contains a float of an additional speed limit
"""


def add_speed_override(speed_override: float):
    """Sets a speed which overrides prior speed"""
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(SPEED_OVERRIDE_ID, speed_override)


def add_speed_limit(speed_limit: float):
    """Sets an additional speed limit"""
    blackboard = py_trees.blackboard.Blackboard()
    current_limit: Optional[float] = blackboard.get(SPEED_LIMIT_ID)
    if current_limit is not None:
        speed_limit = min(speed_limit, current_limit)
    blackboard.set(SPEED_LIMIT_ID, speed_limit)


class SpeedAlterationSetupBehavior(py_trees.behaviour.Behaviour):
    """Sets up the *SPEED_OVERRIDE_ID*
    and *SPEED_LIMIT_ID* in the blackboard to None
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.blackboard.set(SPEED_OVERRIDE_ID, None)
        self.blackboard.set(SPEED_LIMIT_ID, None)

        return py_trees.common.Status.SUCCESS


class SpeedAlterationRequestBehavior(py_trees.behaviour.Behaviour):
    """Reads the speed override and limit from *SPEED_OVERRIDE_ID*
    and *SPEED_LIMIT_ID* requests them from the SpeedAlteration service
    """

    def __init__(self, speed_alteration_client: Client):
        super().__init__(
            type(self).__name__,
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.client = speed_alteration_client

    def update(self):
        req = SpeedAlteration.Request()
        speed_override = self.blackboard.get(SPEED_OVERRIDE_ID)
        speed_limit = self.blackboard.get(SPEED_LIMIT_ID)

        if speed_override is None:
            req.speed_override_active = False
        else:
            req.speed_override_active = True
            req.speed_override = speed_override

        if speed_limit is None:
            req.speed_limit_active = False
        else:
            req.speed_limit_active = True
            req.speed_limit = speed_limit

        self.client.call(req)

        return py_trees.common.Status.SUCCESS

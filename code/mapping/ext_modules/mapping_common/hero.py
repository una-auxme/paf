"""Contains attributes and functions specific to the hero

**[API documentation](/doc/mapping/generated/mapping_common/hero.md)**

Currently available:
- Shape: Length/Width of the Hero based on the Lincoln MKZ 2020
- Function to create a basic hero Entity
"""

from mapping_common.entity import Car, Flags
from mapping_common.shape import Rectangle
from mapping_common.transform import Transform2D, Vector2

# Values based on https://www.motortrend.com/cars/
# lincoln/mkz/2020/specs/?trim=Base+Sedan
HERO_CAR_LENGTH: float = 4.92506
"""Hero car length in meters
"""
HERO_CAR_WIDTH: float = 1.86436
"""Hero car width in meters
"""


def create_hero_entity() -> Car:
    """Creates a car-entity with the shape of the hero car

    Returns:
        Car: hero entity
    """
    shape = Rectangle(
        length=HERO_CAR_LENGTH,
        width=HERO_CAR_WIDTH,
        offset=Transform2D.new_translation(Vector2.new(0.0, 0.0)),
    )
    transform = Transform2D.identity()
    flags = Flags(is_collider=True, is_hero=True)
    hero = Car(
        confidence=1.0,
        priority=1.0,
        shape=shape,
        transform=transform,
        flags=flags,
    )
    return hero

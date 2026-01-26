<!-- markdownlint-disable -->
# Hero documentation

## Table of Contents

* [mapping\_common.hero](#mapping_common.hero)
  * [HERO\_CAR\_LENGTH](#mapping_common.hero.HERO_CAR_LENGTH)
  * [HERO\_CAR\_WIDTH](#mapping_common.hero.HERO_CAR_WIDTH)
  * [create\_hero\_entity](#mapping_common.hero.create_hero_entity)

<a id="mapping_common.hero"></a>

# mapping\_common.hero

Contains attributes and functions specific to the hero

**[API documentation](/doc/mapping/generated/mapping_common/hero.md)**

Currently available:
- Shape: Length/Width of the Hero based on the Lincoln MKZ 2020
- Function to create a basic hero Entity

<a id="mapping_common.hero.HERO_CAR_LENGTH"></a>

#### HERO\_CAR\_LENGTH: `float`

Hero car length in meters

<a id="mapping_common.hero.HERO_CAR_WIDTH"></a>

#### HERO\_CAR\_WIDTH: `float`

Hero car width in meters

<a id="mapping_common.hero.create_hero_entity"></a>

#### create\_hero\_entity

```python
def create_hero_entity() -> Car
```

[[view_source]](/doc/mapping/../../code/mapping/mapping_common/hero.py#L24)

Creates a car-entity with the shape of the hero car

**Returns**:

- `Car` - hero entity


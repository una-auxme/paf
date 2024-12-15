class Obstruction:
    """Obstruction Class for generating the Potential Field around
    dimensions (tuple[float]) width, height
    position (tuple[float]) x,y
    angle (float) degree
    """
    def __init__(self, dimensions: tuple[float] = (1, 2),
                 position: tuple[float] = (2, 2),
                 rotation: float = 0) -> None:

        self.dimensions: tuple[float] = dimensions
        self.position: tuple[float] = position
        self.rotation: float = rotation

    def update_position(self, position: tuple[float], rotation: float):
        """Updates the Position and Rotation of the Obstruction

        Args:
            position (tuple[float]): the Position of the Obstruction
            rotation (float): the roation of the Obstruction
        """
        self.position = position
        self.rotation = rotation

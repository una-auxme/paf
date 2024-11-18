from skimage.draw import polygon
from scipy.ndimage import distance_transform_edt
import numpy as np
import matplotlib.pyplot as plt

from obstruction import Obstruction


class PotentialField:
    """Class for Gernerating a Potential Field around Obstructions

    Its Goal is, to use the Resulting force of a Force field to ensure
    smooth steering around potential obstacles.
    """
    def __init__(self, slope_angle: float,
                 obstructions: list[Obstruction],
                 next_waypoint: tuple[float],
                 dimensions: tuple[float] = (20, 15)) -> None:

        self.slope_angle: float = slope_angle
        self.obstructions: list[Obstruction] = obstructions
        self.DIMENSIONS = [dimension*100 for dimension in dimensions]
        self.field = None
        self.obstruction_map = None

        self.generate_field()

    def add_obstruction(self, obstruction: Obstruction) -> None:
        """adds another obstruction

        Args:
            obstruction (Obstruction): the obstruction to be added
        """
        self.obstructions.append(obstruction)
        self.generate_field()
    
    def get_vector(self, waypoint: tuple[float]) -> tuple[float]:
        """Calcutlates the Direction and Strength of the Resulting
        Force of the Potential Field

        Args:
            position (tuple[float]): the Position where the Strength has to
            be evaluated

        Returns:
            tuple[float]: The "Force" acting at this position
        """

        # Obstacle Heatmap
        distances = distance_transform_edt(self.field == 0)

        k = .001
        smoothed_values = np.exp(-k * distances)
        self.field[self.field == 0] = smoothed_values[self.field == 0]

        # Waypoint Following
        x, y = np.meshgrid(range(self.DIMENSIONS[1]),
                           range(self.DIMENSIONS[0]))

        distances = np.sqrt((x-waypoint[0])**2 + (y-waypoint[1])**2)
        gradient = (distances / distances.max())
        self.field += gradient

        return (0, 0)

    def generate_field(self):
        """generates the field.
        The actor sits at the root of the coordinate system.
        The Obstruction Position is relative to the Actor.
        The Next waypoint position is relative to the actor.
        The Potential Field is generated with the dimensions specified
        in the init.
        """
        if self.field is None:
            # generate the field with a resolution of 1 cm.
            self.field = np.zeros(self.DIMENSIONS, dtype=np.float32)
        
        if self.obstruction_map is None:
            self.obstruction_map = np.zeros(self.DIMENSIONS, dtype=np.float32)

        # place obstructions
        for obstruction in self.obstructions:
            self.place_obstruction(obstruction=obstruction)

    def plot(self, save=False, show=True) -> None:
        """shows the field
        """
        fig, axes = plt.subplots(1, 2)
        axes[0].imshow(self.field, cmap="RdYlGn_r")
        axes[1].imshow(self.obstruction_map, cmap="grey")
        plt.title("Potential Field")
        if save:
            plt.savefig("potential_field.png")
        if show:
            plt.show()

    def place_obstruction(self, obstruction: Obstruction) -> None:
        """places the obstruction in the field

        Args:
            obstruction (Obstruction): the obstruction to place
        """
        # Compute the rectangle's vertices in its local coordinates
        rect_x = np.array([-obstruction.dimensions[0] / 2,
                           obstruction.dimensions[0] / 2,
                           obstruction.dimensions[0] / 2,
                           -obstruction.dimensions[0] / 2])
        rect_y = np.array([-obstruction.dimensions[1] / 2,
                           -obstruction.dimensions[1] / 2,
                           obstruction.dimensions[1] / 2,
                           obstruction.dimensions[1] / 2])

        # Compute rotation matrix
        rad_angle = np.deg2rad(obstruction.rotation)
        rotation_matrix = np.array([
            [np.cos(rad_angle), -np.sin(rad_angle)],
            [np.sin(rad_angle), np.cos(rad_angle)],
        ])

        # Apply rotation
        rotated_vertices = np.dot(rotation_matrix, np.vstack((rect_x, rect_y)))

        rotated_vertices[0] += obstruction.position[0]
        rotated_vertices[1] += obstruction.position[1]

        rr, cc = polygon(rotated_vertices[1],
                         rotated_vertices[0],
                         self.field.shape)
        self.field[rr, cc] = 1
        self.obstruction_map[rr, cc] = 1


if __name__ == "__main__":
    obs = list()
    # car
    obs.append(Obstruction((200, 500), (300, 300), 10))
    
    field = PotentialField(10, obs, (0, 0))
    # lines
    field.add_obstruction(
        Obstruction(position=(field.DIMENSIONS[1], field.DIMENSIONS[0]//2),
                    dimensions=(20, field.DIMENSIONS[0])))
    field.add_obstruction(
        Obstruction(position=(field.DIMENSIONS[1]//2, field.DIMENSIONS[0]//2),
                    dimensions=(20, field.DIMENSIONS[0])))
    field.get_vector(waypoint=(field.DIMENSIONS[0]//2, 0))
    field.plot(save=True, show=False)

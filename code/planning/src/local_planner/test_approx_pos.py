import unittest
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def approx_obstacle_pos(distance: float, heading: float, ego_pos: np.array):
    """calculate the position of the obstacle in the global coordinate system
        based on ego position, heading and distance
    """
    rotation_matrix = Rotation.from_euler('z', heading).as_matrix()

    # Annahme: Relative Position des vorausfahrenden Fahrzeugs in Ihrem lokalen Koordinatensystem
    relative_position_local = np.array([0, distance, 0])

    # Schritt 1: Rotation auf die relative Position anwenden
    absolute_position_local = rotation_matrix.dot(relative_position_local)

    # Schritt 2: Absolute Position in das globale Koordinatensystem transformieren
    vehicle_position_global = ego_pos + absolute_position_local
    return vehicle_position_global


class TestMotionPlanning(unittest.TestCase):
    def test_approx_obstacle_pos(self):
        ego_pos = np.array([0, 0, 0])
        target_positions = [np.array([10, 10, 0]), np.array([20, 20, 0]), np.array([30, 30, 0])]
        headings = [0, np.pi/2, np.pi, 3*np.pi/2]

        plt.figure()
        plt.title('Approximated vs Expected Positions')
        plt.xlabel('x')
        plt.ylabel('y')

        for target_position in target_positions:
            for heading in headings:
                distance = np.linalg.norm(target_position - ego_pos)
                calculated_position = approx_obstacle_pos(distance, heading, ego_pos)
                np.testing.assert_array_almost_equal(calculated_position, target_position, decimal=5)

                plt.scatter(*target_positions[:2], color='blue', label='Expected')
                plt.scatter(*calculated_position[:2], color='red', label='Calculated')

        plt.legend()
        plt.show()


if __name__ == '__main__':
    unittest.main()
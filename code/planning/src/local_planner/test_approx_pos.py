import unittest
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import math


def approx_obstacle_pos(distance: float, heading: float, ego_pos: np.array):
    """calculate the position of the obstacle in the global coordinate system
        based on ego position, heading and distance
    """
    rotation_matrix = Rotation.from_euler('z', heading)

    # Annahme: Relative Position des vorausfahrenden Fahrzeugs in Ihrem lokalen Koordinatensystem
    relative_position_local = np.array([0, distance, 0])

    # Schritt 1: Rotation auf die relative Position anwenden
    absolute_position_local = rotation_matrix.apply(relative_position_local)

    # Schritt 2: Absolute Position in das globale Koordinatensystem transformieren
    vehicle_position_global = ego_pos + absolute_position_local
    return vehicle_position_global


class TestMotionPlanning(unittest.TestCase):
    def test_approx_obstacle_pos(self):
        ego_pos = np.array([0, 0, 0])
        target_positions = np.array([np.array([10, 10, 0]), np.array([20, 10, 0]), np.array([30, 30, 0])])
        headings = [math.radians(315), -0.4636476, math.radians(315)]

        plt.figure()
        plt.title('Approximated vs Expected Positions')
        plt.xlabel('x')
        plt.ylabel('y')

        for index in range(len(target_positions)):
            distance = np.linalg.norm(target_positions[index] - ego_pos)
            calculated_position = approx_obstacle_pos(distance, headings[index], ego_pos)
            print(calculated_position)
            # np.testing.assert_array_almost_equal(calculated_position, target_position, decimal=1)

            plt.scatter(*target_positions[:2], color='blue', label='Expected')
            plt.scatter(*calculated_position[:2], color='red', label='Calculated', marker='x')

        plt.legend(loc='lower left')
        plt.grid(visible=True)
        plt.show()


if __name__ == '__main__':
    unittest.main()

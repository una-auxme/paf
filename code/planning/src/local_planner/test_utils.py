import unittest
import numpy as np
from utils import filter_vision_objects


class TestUtils(unittest.TestCase):
    def test_filter_vision_objects(self):
        # Create a sample input data
        data = np.array([[1, 2.5, 1.0, 2.0, 3.0, 4.0, 0.2],
                         [2, 3.5, 2.0, 3.0, 4.0, 5.0, -1.0],
                         [2, 4.5, 3.0, 4.0, 5.0, 6.0, -2.0],
                         [3, 5.5, 4.0, 5.0, 6.0, 7.0, 0.8]])

        # Call the method under test
        result = filter_vision_objects(data)

        # Define the expected output
        expected_output = np.array([2, 3.5, 2.0, 3.0, 4.0, 5.0, -1.0])

        # Assert that the result matches the expected output
        np.testing.assert_array_equal(result, expected_output)


if __name__ == '__main__':
    unittest.main()
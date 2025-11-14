import numpy as np
import pytest
from typing import Dict, Any

from perception.lidar_distance import create_transform_matrix, create_delta_matrix, ego_motion_compensation 


# --- Mock Data Structures ---

class MockVector3:
    """A minimal mock for ROS Vector3."""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MockQuaternion:
    """A minimal mock for ROS Quaternion."""
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class MockPose:
    """Simulates geometry_msgs.msg.Pose, holding position and orientation data."""
    def __init__(self, translation, rotation):
        self.position = MockVector3(**translation)
        self.orientation = MockQuaternion(**rotation)

class MockPoseStamped:
    """Simulates geometry_msgs.msg.PoseStamped, the primary input format."""
    def __init__(self, translation, rotation):
        self.pose = MockPose(translation, rotation)


# ----------------------------------------------------
# FIXTURES (Test Data Scenarios)
# ----------------------------------------------------

@pytest.fixture
def pos_A():
    """Start State (Previous Pose): Identity pose at (0, 0, 0)."""
    return MockPoseStamped(
        translation={'x': 0.0, 'y': 0.0, 'z': 0.0},
        rotation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    )

@pytest.fixture
def pos_B_translation():
    """Current State: Pure translation 1 meter along the X-axis (forward movement)."""
    return MockPoseStamped(
        translation={'x': 1.0, 'y': 0.0, 'z': 0.0},
        rotation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    )

@pytest.fixture
def pos_C_rotation():
    """Current State: Pure Z-axis rotation by 90 degrees counter-clockwise (turning left)."""
    # Quaternion representing 90 degrees rotation around Z
    return MockPoseStamped(
        translation={'x': 0.0, 'y': 0.0, 'z': 0.0},
        rotation={'x': 0.0, 'y': 0.0, 'z': 0.70710678, 'w': 0.70710678} 
    )

@pytest.fixture
def old_points_array():
    """A single point located 10m directly ahead of the car (X=10.0) in the previous frame."""
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.uint8)])
    return np.array([(10.0, 0.0, 0.0, 50)], dtype=dtype)

@pytest.fixture
def point_on_y_axis():
    """A single point located 10m to the side (Y=10.0) for rotation tests."""
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.uint8)])
    return np.array([(0.0, 10.0, 0.0, 50)], dtype=dtype)


# ----------------------------------------------------
# TEST FUNCTIONS
# ----------------------------------------------------

def test_create_transform_matrix(pos_A, pos_B_translation, pos_C_rotation):
    """Verifies that the 4x4 homogeneous transformation matrix is created correctly from a Pose."""
    
    # Test 1: Identity Pose
    # Expect: An identity matrix (no translation or rotation).
    T_A = create_transform_matrix(pos_A)
    assert np.allclose(T_A, np.eye(4))

    # Test 2: Pure Translation
    # Expect: Translation vector [1.0, 0.0, 0.0] in the matrix's translation column.
    T_B = create_transform_matrix(pos_B_translation)
    assert np.allclose(T_B[:3, 3], [1.0, 0.0, 0.0])

    # Test 3: Pure Rotation (90 degrees CCW around Z)
    # Expect: The rotation block (top-left 3x3) should represent a 90-degree Z-rotation.
    T_C = create_transform_matrix(pos_C_rotation)
    R_expected = np.array([
        [0.0, -1.0, 0.0], 
        [1.0, 0.0, 0.0],  
        [0.0, 0.0, 1.0]
    ])
    assert np.allclose(T_C[:3, :3], R_expected, atol=1e-5)

def test_create_delta_matrix(pos_A, pos_B_translation, pos_C_rotation):
    """Verifies the Delta Matrix (dT) correctly encodes the *compensation* motion needed (T_compensation)."""
    
    # Test 1: Pure Translation A -> B (1m forward motion)
    dT_trans = create_delta_matrix(pos_B_translation, pos_A)
    T_expected_delta = np.eye(4)
    # Expect: dT should contain a -1.0m translation to *undo* the car's 1.0m forward motion.
    T_expected_delta[:3, 3] = [-1.0, 0.0, 0.0]
    assert np.allclose(dT_trans, T_expected_delta)

    # Test 2: Pure Rotation A -> C (90 degrees CCW turn)
    dT_rot = create_delta_matrix(pos_C_rotation, pos_A)
    # Expect: dT must be the inverse rotation matrix (90 degrees CW rotation) to compensate the turn.
    T_C_inv = np.linalg.inv(create_transform_matrix(pos_C_rotation))
    assert np.allclose(dT_rot, T_C_inv)


def test_ego_motion_compensation(pos_A, pos_B_translation, old_points_array, pos_C_rotation, point_on_y_axis):
    """Verifies that applying the compensation matrix (dT) shifts the old point cloud correctly into the current sensor frame."""
    
    # --- Test 1: Translation Compensation ---
    # Car moved 1.0m forward. dT applies -1.0m compensation.
    # The point at X=10.0 (old frame) should now appear at X=9.0 (new frame).
    dT_trans = create_delta_matrix(pos_B_translation, pos_A)
    compensated_trans_arr = ego_motion_compensation(old_points_array, dT_trans)
    
    expected_x_trans = 9.0
    actual_x_trans = compensated_trans_arr['x'][0]
    
    assert np.isclose(actual_x_trans, expected_x_trans)
    assert np.isclose(compensated_trans_arr['y'][0], 0.0)


    # --- Test 2: Rotation Compensation ---
    # Car rotated 90 degrees CCW. dT applies 90 degrees CW compensation (T_C_inv).
    # Point at (0, 10, 0) is rotated 90 degrees CW, landing on the positive X-axis.
    dT_rot = create_delta_matrix(pos_C_rotation, pos_A)
    compensated_rot_arr = ego_motion_compensation(point_on_y_axis, dT_rot)

    expected_x_rot = 10.0 
    expected_y_rot = 0.0
    
    assert np.isclose(compensated_rot_arr['x'][0], expected_x_rot, atol=1e-5)
    assert np.isclose(compensated_rot_arr['y'][0], expected_y_rot, atol=1e-5)
    assert compensated_rot_arr['intensity'][0] == 50
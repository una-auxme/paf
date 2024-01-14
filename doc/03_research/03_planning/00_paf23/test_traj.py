from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.fot_wrapper import run_fot
import numpy as np
import matplotlib.pyplot as plt

wp = wp = np.r_[[np.full((50), 983.5889666959667)], [np.linspace(5370.016106881272, 5399.016106881272, 50)]].T
initial_conditions = {
    'ps': 0,
    'target_speed': 6,
    'pos': np.array([983.5807552562393, 5370.014637890163]),
    'vel': np.array([5, 1]),
    'wp': wp,
    'obs': np.array([[983.568124548765, 5386.0219828457075,
             983.628124548765, 5386.0219828457075]])
}

hyperparameters = {
    "max_speed": 25.0,
    "max_accel": 15.0,
    "max_curvature": 15.0,
    "max_road_width_l": 3.0,
    "max_road_width_r": 0,
    "d_road_w": 0.5,
    "dt": 0.2,
    "maxt": 20.0,
    "mint": 6.0,
    "d_t_s": 0.5,
    "n_s_sample": 2.0,
    "obstacle_clearance": 2,
    "kd": 1.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.1,
    "kt": 0.1,
    "ko": 0.1,
    "klat": 1.0,
    "klon": 1.0,
    "num_threads": 0,  # set 0 to avoid using threaded algorithm
}

result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = run_fot(initial_conditions,
                                                     hyperparameters)

if success:
    print("Success!")
    print("result_x: ", result_x)
    print("result_y: ", result_y)
    fig, ax = plt.subplots(1, 2)

    ax[0].scatter(wp[:,0], wp[:,1], label="original")
    ax[0].scatter([983.568124548765, 983.628124548765], [5386.0219828457075, 5386.0219828457075], label="object")
    ax[0].set_xticks([983.518124548765, 983.598124548765])
    ax[1].scatter(result_x, result_y, label="frenet")
    ax[1].scatter([983.568124548765, 983.628124548765], [5386.0219828457075, 5386.0219828457075], label="object")
    ax[1].set_xticks([983.518124548765, 983.598124548765])
    plt.legend()
    plt.show()
else:
    print("Failure!")

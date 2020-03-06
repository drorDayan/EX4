# Metrics weights:
weight_distance_sum = 0.0001
weight_distance_max = 0
weight_distance_pyt = 0
weight_robot_clearance_inv = 1
weight_turn_angle_max = 0  # Experimental, not working as intended...

# Running time:
seconds_to_run = 60
seconds_between_samples = 10
exit_on_success = True          # If set to True, will not try to optimize further once a path has been found
continue_until_success = True   # If set to True, will not quit if seconds_to_run seconds have passed

# Algorithm Variables:
k_nearest = 10
steer_eta = 1.0
num_of_points_in_batch = 1200

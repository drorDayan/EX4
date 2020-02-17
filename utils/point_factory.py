import random
from arr2_epec_seg_ex import *
from utils.misc import interweave


# TODO assume sqr and use [blah for range]
def get_batch(robot_num, num_of_points, max_x, max_y, min_x, min_y, dest_point):
    v = []
    num_of_points_in_dest_direction = random.randint(0, num_of_points / 5)
    for i in range(num_of_points - num_of_points_in_dest_direction):
        coordinates_x = [FT(random.uniform(min_x, max_x)) for i in range(robot_num)]
        coordinates_y = [FT(random.uniform(min_y, max_y)) for i in range(robot_num)]
        coordinates = interweave(coordinates_x, coordinates_y)
        v.append(Point_d(2 * robot_num, coordinates))
    # we should try and steer to goal with some probability
    for i in range(num_of_points_in_dest_direction):
        coordinates_x = [(FT(random.uniform(min_x, max_x)) + dest_point[2 * i]) / FT(2) for i in range(robot_num)]
        coordinates_y = [(FT(random.uniform(min_y, max_y)) + dest_point[2 * i + 1]) / FT(2) for i in range(robot_num)]
        coordinates = interweave(coordinates_x, coordinates_y)
        v.append(Point_d(2 * robot_num, coordinates))
    return v

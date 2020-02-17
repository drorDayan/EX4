from arr2_epec_seg_ex import *


def get_min_max(obstacles):
    max_x = max(max(v.x() for v in obs) for obs in obstacles)
    max_y = max(max(v.y() for v in obs) for obs in obstacles)
    min_x = min(min(v.x() for v in obs) for obs in obstacles)
    min_y = min(min(v.y() for v in obs) for obs in obstacles)
    return max_x.to_double(), max_y.to_double(), min_x.to_double(), min_y.to_double()


def get_square_mid(robot):
    x = (robot[0].x() + robot[1].x()) / FT(2)
    y = (robot[1].y() + robot[2].y()) / FT(2)
    return [x, y]


def get_origin_robot_coord(width):
    robot_width = width / FT(2)
    v1 = Point_2(robot_width, robot_width)
    v2 = Point_2(robot_width * FT(-1), robot_width)
    v3 = Point_2(robot_width * FT(-1), robot_width * FT(-1))
    v4 = Point_2(robot_width, robot_width * FT(-1))
    return [v1, v2, v3, v4]

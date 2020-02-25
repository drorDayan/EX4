from arr2_epec_seg_ex import *


def get_normal_movement_vector(p1, p2, i, j):
    i_x_diff = p2[2 * i] - p1[2 * i]
    i_y_diff = p2[2 * i + 1] - p1[2 * i + 1]
    j_x_diff = p2[2 * j] - p1[2 * j]
    j_y_diff = p2[2 * j + 1] - p1[2 * j + 1]
    start_point = [p1[2 * i] - p1[2 * j], p1[2 * i + 1] - p1[2 * j + 1]]
    diff_vec = [i_x_diff - j_x_diff, i_y_diff - j_y_diff]
    normal_movement_vector = Curve_2(Point_2(start_point[0], start_point[1]),
                                     Point_2(start_point[0] + diff_vec[0], start_point[1] + diff_vec[1]))
    return normal_movement_vector


def two_robot_intersect(p1, p2, i, j, double_width_square_arrangement, double_width_square_point_locator):
    mov_vec = get_normal_movement_vector(p1, p2, i, j)
    return do_intersect(double_width_square_arrangement, mov_vec)


# checks for collisions return:
# True if collision free
# False, first robot index that touches an obs if a robot touches an obs
# False, robot_num if a robot touches another robot
def path_collision_free(robot_num, p1, p2, arrangement, double_width_square_arrangement,
                        double_width_square_point_locator):
    # check for obs collision
    for i in range(robot_num):
        if do_intersect(arrangement, Curve_2(Point_2(p1[2 * i], p1[2 * i + 1]), Point_2(p2[2 * i], p2[2 * i + 1]))):
            return False
    # check for robot to robot collision
    for i in range(robot_num):
        for j in range(i + 1, robot_num):
            if two_robot_intersect(p1, p2, i, j, double_width_square_arrangement,
                                   double_width_square_point_locator):
                return False
    return True

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
    # zone_output = []
    # zone(double_width_square_arrangement, mov_vec, zone_output, double_width_square_point_locator)
    # if len(zone_output) > 1:
    #     return True
    # return False


# checks for collisions return:
# True if collision free
# False, first robot index that touches an obs if a robot touches an obs
# False, robot_num if a robot touches another robot
def path_collision_free(point_locator, robot_num, p1, p2, arrangement, double_width_square_arrangement,
                        double_width_square_point_locator, do_single=False, robot_idx=0, first_invalid_idx=0):
    # check for obs collision
    if do_single:
        if robot_idx < first_invalid_idx:
            robots_to_check = []
        elif robot_idx > first_invalid_idx:
            robots_to_check = [robot_idx]
        else:
            return False, robot_idx
    else:
        robots_to_check = [i for i in range(robot_num)]
    for i in robots_to_check:
        if do_intersect(arrangement, Curve_2(Point_2(p1[2 * i], p1[2 * i + 1]), Point_2(p2[2 * i], p2[2 * i + 1]))):
            return False, i
        # zone_output = []
        # zone(arrangement, Curve_2(Point_2(p1[2*i], p1[2*i+1]), Point_2(p2[2*i], p2[2*i+1])),
        #      zone_output, point_locator)
        # if len(zone_output) > 1:
        #     return False, i
    # check for robot to robot collision
    if not do_single:
        for i in range(robot_num):
            for j in range(i + 1, robot_num):
                if two_robot_intersect(p1, p2, i, j, double_width_square_arrangement,
                                       double_width_square_point_locator):
                    return False, robot_num
    else:
        for j in range(robot_num):
            if j == robot_idx:
                continue
            if two_robot_intersect(p1, p2, robot_idx, j, double_width_square_arrangement,
                                   double_width_square_point_locator):
                return False, robot_num
    return True, 0

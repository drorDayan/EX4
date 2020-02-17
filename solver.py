import random
import time
from utils.rrt_node import *
from config import *
from utils.misc import interweave
from utils.polygons_arrangement import *
from utils.collision_detection import *
from utils.neighbor_finder import *
from utils.point_factory import *
from utils.scene_data import *

# Code: #


def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(robot_num, near, rand).to_double()))
    if dist < eta:
        return rand
    else:
        return Point_d(2 * robot_num, [near[i] + (rand[i] - near[i]) * eta / dist for i in range(2 * robot_num)])


def try_connect_to_dest(graph, point_locator, robot_num, tree, dest_point, arrangement, double_width_square_arrangement,
                        double_width_square_point_locator):
    nn = k_nn(tree, k_nearest, dest_point, FT(0))
    for neighbor in nn:
        free, x = path_collision_free(point_locator, robot_num, neighbor[0], dest_point, arrangement,
                                      double_width_square_arrangement, double_width_square_point_locator)
        if free:
            graph[dest_point] = RRT_Node(dest_point, robot_num, graph[neighbor[0]])
            return True
    return False


def generate_path(path, robots, obstacles, destination):
    # random.seed(0) #  for tests
    start = time.time()
    robot_num = len(robots)
    assert (len(destination) == robot_num)
    for i in range(robot_num):
        robot_width = robots[i][1].x() - robots[i][0].x()
        assert (robot_width == FT(1))
    do_use_single_robot_movement = False
    # init obs for collision detection
    one_width_square = Polygon_2(get_origin_robot_coord(robot_width))
    double_width_square = Polygon_with_holes_2(Polygon_2(get_origin_robot_coord(FT(2) * robot_width)))
    inflated_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
    c_space_obstacles = [minkowski_sum_by_full_convolution_2(one_width_square, obs) for obs in inflated_obstacles]
    c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
    obstacles_arrangement = overlay_multiple_arrangements(c_space_arrangements, merge_faces_by_freespace_flag)
    obstacles_point_locator = Arr_trapezoid_ric_point_location(obstacles_arrangement)
    double_width_square_arrangement = polygon_with_holes_to_arrangement(double_width_square)
    double_width_square_point_locator = Arr_trapezoid_ric_point_location(double_width_square_arrangement)

    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    start_ref_points = [get_square_mid(robot) for robot in robots]
    target_ref_points = [[dest.x(), dest.y()] for dest in destination]
    start_point = Point_d(2 * robot_num, sum(start_ref_points, []))
    dest_point = Point_d(2 * robot_num, sum(target_ref_points, []))
    vertices = [start_point]
    graph = {start_point: RRT_Node(start_point, robot_num)}
    tree = Kd_tree(vertices)
    while True:
        print("new batch, time= ", time.time() - start)
        # I use a batch so that the algorithm can be iterative
        batch = get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point)
        new_points = []
        for p in batch:
            near = get_nearest(robot_num, tree, new_points, p)
            new = steer(robot_num, near, p, FT(steer_eta))
            free, idx = path_collision_free(obstacles_point_locator, robot_num, near, new, obstacles_arrangement,
                                            double_width_square_arrangement, double_width_square_point_locator)
            if free:
                new_points.append(new)
                vertices.append(new)
                graph[new] = RRT_Node(new, robot_num, graph[near])
            elif do_use_single_robot_movement:
                for i in range(robot_num):
                    new_data = [near[j] for j in range(2 * robot_num)]
                    new_data[2 * i] = new[2 * i]
                    new_data[2 * i + 1] = new[2 * i + 1]
                    my_new = Point_d(2 * robot_num, new_data)
                    free, aa = path_collision_free(obstacles_point_locator, robot_num, near, my_new,
                                                   obstacles_arrangement, double_width_square_arrangement,
                                                   double_width_square_point_locator, do_single=True, robot_idx=i,
                                                   first_invalid_idx=idx)
                    if free:
                        new_points.append(my_new)
                        vertices.append(my_new)
                        graph[my_new] = RRT_Node(my_new, robot_num, graph[near])

        # this in in-efficient if this becomes a bottleneck we should hold an array of kd-trees
        # each double the size of the previous one
        tree.insert(new_points)
        print("vertices amount: ", len(vertices))
        if len(new_points) < single_robot_movement_if_less_then:
            # print("single robot movement")
            do_use_single_robot_movement = use_single_robot_movement
        if try_connect_to_dest(graph, obstacles_point_locator, robot_num, tree, dest_point, obstacles_arrangement,
                               double_width_square_arrangement, double_width_square_point_locator):
            break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2 * i], dp[2 * i + 1]) for i in range(robot_num)])
    # print("k_nearest = ", k_nearest)
    # print("steer_eta = ", steer_eta)
    # print("num_of_points_in_batch = ", num_of_points_in_batch)
    # print("used single robot movement:", do_use_single_robot_movement)
    print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)
    print(graph[dest_point].calc_value())

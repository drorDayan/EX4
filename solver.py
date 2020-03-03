import time
from utils.rrt_node import *
from config import *
from utils.polygons_arrangement import *
from utils.collision_detection import *
from utils.neighborhood import *
from utils.point_factory import *
from utils.scene_data import *


# Code: #


def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(near, rand).to_double()))
    if dist < eta:
        return rand, False
    else:
        return Point_d(2 * robot_num, [near[i] + (rand[i] - near[i]) * eta / dist for i in range(2 * robot_num)]), True


def try_connect_to_dest(graph, point_locator, robot_num, tree, dest_point, arrangement, double_width_square_arrangement,
                        double_width_square_point_locator):
    nn = k_nn(tree, k_nearest, dest_point, FT(0))
    for neighbor in nn:
        if path_collision_free(robot_num, neighbor[0], dest_point, arrangement, double_width_square_arrangement,
                               double_width_square_point_locator):
            graph[dest_point] = RRT_Node(dest_point, graph[neighbor[0]])
            return True
    return False


def generate_path(path, robots, obstacles, destination):
    robot_num = len(robots)
    assert (len(destination) == robot_num)
    robot_width = 0
    for i in range(robot_num):
        robot_width = robots[i][1].x() - robots[i][0].x()
        assert (robot_width == FT(1))
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
    destination_point = Point_d(2 * robot_num, sum(target_ref_points, []))
    vertices = [start_point]
    graph = {start_point: RRT_Node(start_point)}
    tree = Kd_tree(vertices)
    start_time = time.time()
    time_of_last_sample = start_time
    connected_to_destination = False
    just_succeeded_connecting = False
    samples = []
    while True:
        current_time = time.time()
        if current_time - start_time >= seconds_to_run:
            break

        trying_destination = False
        if current_time - time_of_last_sample >= seconds_per_sample or just_succeeded_connecting:
            time_of_last_sample = current_time
            if not connected_to_destination:
                print("trying to connect to destination, time= ", current_time - start_time)
                batch = [destination_point]
                trying_destination = True
            else:
                destination_node = graph[destination_point]
                samples.append((current_time-start_time, destination_node.path_to_origin_values_dict()))
            just_succeeded_connecting = False
        if not trying_destination:
            print("new batch, time= ", current_time - start_time)
            # I use a batch so that the algorithm can be iterative
            batch = get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, destination_point)

        # We add points to the tree in batches, so we hold the new points in an array and pass them with the tree
        # when looking for neighbors
        new_points = []
        for p in batch:
            nearest_point = get_nearest(robot_num, tree, new_points, p)
            steered_point, steered = steer(robot_num, nearest_point, p, FT(steer_eta))
            if path_collision_free(robot_num, nearest_point, steered_point, obstacles_arrangement,
                                   double_width_square_arrangement, double_width_square_point_locator):
                if trying_destination and not steered:
                    connected_to_destination = True
                    just_succeeded_connecting = True
                    print("Successfully found a path to the destination!")
                # Look for best neighbor:
                neighborhood = find_neighborhood(robot_num, tree, new_points, steered_point, FT(steer_eta))
                new_points.append(steered_point)
                vertices.append(steered_point)
                new_node = RRT_Node(steered_point)
                if nearest_point in neighborhood:  # make sure that nearest_point is first
                    neighborhood.remove(nearest_point)
                neighborhood.insert(0, nearest_point)
                neighborhood_values = {}
                for neighbor in neighborhood:
                    if path_collision_free(robot_num, neighbor, steered_point, obstacles_arrangement,
                                           double_width_square_arrangement, double_width_square_point_locator):
                        neighborhood_values[neighbor] = new_node.path_to_origin_through_target_values(graph[neighbor])
                best_neighbor = min(neighborhood_values, key=lambda k: neighborhood_values.get(k)[0])
                new_node.set_parent(graph[best_neighbor])
                graph[steered_point] = new_node
                # re-wiring:
                for neighbor in neighborhood_values.keys():  # Only neighbors with free paths were added
                    if neighbor == best_neighbor:
                        continue
                    if neighborhood_values[neighbor][1] >\
                            graph[neighbor].path_to_origin_through_target_values(new_node)[0]:
                        graph[neighbor].set_parent(new_node)
        # this in in-efficient if this becomes a bottleneck we should hold an array of kd-trees
        # each double the size of the previous one
        tree.insert(new_points)
        print("vertices amount: ", len(vertices))
    d_path = []
    graph[destination_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2 * i], dp[2 * i + 1]) for i in range(robot_num)])
    print("finished, time= ", time.time() - start_time, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)
    for sample in samples:
        print(str(sample[0]).ljust(20, '0'),
              '\t',
              str(sample[1]).ljust(8, '0'),
              '\t',
              '\t'.join([str(RRT_Node.costs[cost].extract_value(sample[1][cost]))
                         for cost in RRT_Node.costs.keys()
                         if RRT_Node.costs[cost].weight > 0]))

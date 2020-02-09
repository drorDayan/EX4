from arr2_epec_seg_ex import *
import random
import time
from math import sqrt

# Configurable Variables: #

k_nearest = 10
steer_eta = FT(0.6)
num_of_points_in_batch = 1200
single_robot_movement_if_less_then = 20
use_single_robot_movement = True
FREESPACE = 'freespace'


# Code: #
class RRT_Cost(object):
    def __init__(self, weight, calc_default_func, calc_func, merge_func):
        self.weight = weight
        self.calc_default_func = calc_default_func
        self.calc_func = calc_func
        self.merge_func = merge_func


class RRT_Node(object):
    costs = {
        #  "name":          RRT_Cost(weight,
        #                            lambda c: <defualt value at point c>
        #                            lambda c, p: <value for segment from p to c>,
        #                            lambda old, update : <value for entire path where update is the value for the last step and old is for the rest>
        #                            )
        "distance": RRT_Cost(1,
                             lambda c: FT(0),
                             lambda c, p: distance_squared(c.robot_num, p.point, c.point),
                             lambda old, update: old + update
                             ),
        "clearance_o": RRT_Cost(0,
                                lambda c: calc_clearance_with_obstacles_stationary(c.robot_num, c.point),
                                lambda c, p: calc_clearance_with_obstacles(c.robot_num, p.point, c.point),
                                lambda old, update: min(old, update)
                                ),
        "clearance_r": RRT_Cost(0,
                                lambda c: min_inter_robot_distance_stationary(c.robot_num, c.p),
                                lambda c, p: min_inter_robot_distance(c.robot_num, p.point, c.point),
                                lambda old, update: min(old, update)
                                )
    }

    def __init__(self, pt, robot_num, pr=None):
        self.point = pt
        self.robot_num = robot_num
        self.metrics = {}
        self.parent = None
        self.set_parent(pr)

    def set_parent(self, pr):
        self.parent = pr
        for k in self.costs.keys():
            if self.costs[k].weight > 0:
                if pr is not None:
                    self.metrics[k] = self.costs[k].merge_func(self.parent.metrics[k],
                                                               self.costs[k].calc_func(self, self.parent))
                else:
                    self.metrics[k] = self.costs[k].calc_default_func(self)

    def update_costs(self):
        pass

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path


# Clearance related code: #
def between(s, p, f):
    return (s <= p <= f) or (f <= p <= s)


def min_dist_between_moving_robots_l_inf(s1, s2, t1, t2):
    # 2DO Enumerate over all Gods, Goddesses, Kami, etc. and for each one pray that this code works.
    sx = s1[0] - s2[0]
    sy = s1[1] - s2[1]

    tx = t1[0] - t2[0]
    ty = t1[1] - t2[1]

    dx = tx - sx
    dy = ty - sy

    m = dy / dx
    n = (dx * sy - dy * sx) / dx
    candidates = [max(abs(sx), abs(sy)), max(abs(tx), abs(ty))]
    if m != 1:
        z = n / (1 - m)
        if between(sx, z, tx):
            candidates.append(abs(z))
    if m != -1:
        z = n / (-1 - m)
        if between(sx, z, tx):
            candidates.append(abs(z))
    return min(candidates)


def min_inter_robot_distance(robot_num, start_point, target_point):
    min_dist = None
    for i in range(robot_num):
        s1 = (start_point[2 * i].to_double(), start_point[2 * i + 1].to_double())
        t1 = (target_point[2 * i].to_double(), target_point[2 * i + 1].to_double())
        for j in range(i + 1, robot_num):
            s2 = (start_point[2 * j].to_double(), start_point[2 * j + 1].to_double())
            t2 = (target_point[2 * j].to_double(), target_point[2 * j + 1].to_double())

            new_min_dist = min_dist_between_moving_robots_l_inf(s1, s2, t1, t2)
            min_dist = new_min_dist if min_dist is None else min(min_dist, new_min_dist)

    return min_dist


def min_inter_robot_distance_stationary(robot_num, start_point):
    return 0


def calc_clearance_with_obstacles(robot_num, start_point, target_point):
    return 0


def calc_clearance_with_obstacles_stationary(robot_num, start_point):
    return 0


# obs collision detection code: #

def polygon_with_holes_to_arrangement(poly):
    assert isinstance(poly, Polygon_with_holes_2)
    arr = Arrangement_2()
    insert(arr, [Curve_2(e) for e in poly.outer_boundary().edges()])

    # set the freespace flag for the only current two faces
    for f in arr.faces():
        assert isinstance(f, Face)
        f.set_data({FREESPACE: f.is_unbounded()})

    # TODO: test this with a complex polygon
    for hole in poly.holes():
        insert(arr, [Curve_2(e) for e in hole.edges()])

    for f in arr.faces():
        assert isinstance(f, Face)
        if f.data() is None:
            f.set_data({FREESPACE: True})
    return arr


def merge_faces_by_freespace_flag(x, y):
    return {FREESPACE: x[FREESPACE] and y[FREESPACE]}


def overlay_multiple_arrangements(arrs, face_merge_func):
    final_arr = arrs[0]
    for arr in arrs[1:]:
        temp_res = Arrangement_2()

        overlay(final_arr, arr, temp_res, Arr_face_overlay_traits(face_merge_func))
        final_arr = temp_res
    return final_arr


def is_in_free_face(point_locator, point):
    face = Face()
    # locate can return a vertex or an edge or a face
    located_obj = point_locator.locate(point)
    # if located_obj.is_vertex():
    #     return False
    # if located_obj.is_halfedge():
    #     return False
    if located_obj.is_face():
        located_obj.get_face(face)
        return face.data()[FREESPACE]
    return False


interweave = lambda l1, l2: [m for pair in zip(l1, l2) for m in pair]


# TODO assume sqr and use [blah for range]
def get_batch(robot_num, num_of_points, max_x, max_y, min_x, min_y, dest_point):
    v = []
    num_of_points_in_dest_direction = random.randint(0, num_of_points / 5)
    for i in range(num_of_points - num_of_points_in_dest_direction):
        coords_x = [FT(random.uniform(min_x, max_x)) for i in range(robot_num)]
        coords_y = [FT(random.uniform(min_y, max_y)) for i in range(robot_num)]
        coords = interweave(coords_x, coords_y)
        v.append(Point_d(2 * robot_num, coords))
    # we should try and steer to goal with some probability
    for i in range(num_of_points_in_dest_direction):
        coords_x = [(FT(random.uniform(min_x, max_x)) + dest_point[2 * i]) / FT(2) for i in range(robot_num)]
        coords_y = [(FT(random.uniform(min_y, max_y)) + dest_point[2 * i + 1]) / FT(2) for i in range(robot_num)]
        coords = interweave(coords_x, coords_y)
        v.append(Point_d(2 * robot_num, coords))
    return v


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


def k_nn(tree, k, query, eps):
    search_nearest = True
    sort_neighbors = True
    # print("pre search")
    search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    # print("post search")
    lst = []
    search.k_neighbors(lst)
    return lst


def distance_squared(robot_num, p1, p2):
    # return transformed_distance(p1, p2)
    tmp = FT(0)
    for i in range(2 * robot_num):
        tmp = tmp + (p1[i] - p2[i]) * (p1[i] - p2[i])
    return tmp


def get_nearest(robot_num, tree, new_points, rand):
    nn = k_nn(tree, 1, rand, FT(0))
    nn_in_tree = nn[0]
    if len(new_points) == 0:
        return nn_in_tree[0]
    # check distance from new points
    #  TODO make sure it works
    dist = [distance_squared(robot_num, rand, point) for point in new_points]
    min_dist = dist[0]
    min_i = 0
    for i in range(len(new_points)):
        if dist[i] < min_dist:
            min_dist = dist[i]
            min_i = i
    if min_dist < nn_in_tree[1] * nn_in_tree[1]:
        return new_points[min_i]
    return nn_in_tree[0]


def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(robot_num, near, rand).to_double()))
    if dist < eta:
        return rand
    else:
        return Point_d(2 * robot_num, [near[i] + (rand[i] - near[i]) * eta / dist for i in range(2 * robot_num)])


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


def get_origin_robot_coord(width):
    robot_width = width / FT(2)
    v1 = Point_2(robot_width, robot_width)
    v2 = Point_2(robot_width * FT(-1), robot_width)
    v3 = Point_2(robot_width * FT(-1), robot_width * FT(-1))
    v4 = Point_2(robot_width, robot_width * FT(-1))
    return [v1, v2, v3, v4]


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
            new = steer(robot_num, near, p, steer_eta)
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
    print(graph[dest_point].metrics)

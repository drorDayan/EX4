from arr2_epec_seg_ex import *
import random
import time
from math import sqrt

# Configurable Variables: #

k_nearest = 50
steer_eta = FT(0.3)
inflation_epsilon = FT(0.02)
num_of_points_in_batch = 1000
use_single_robot_movement = True
FREESPACE = 'freespace'

# Code: #
class RRT_Node():
    def __init__(self, pt, pr=None, n=0):
        self.point = pt
        self.parent = pr
        self.nval = n # Will be used to store distance to this point, etc.

    def get_path_to_here(self, ret_path):
        cur = self
        while cur != None:
            ret_path.insert(0, cur.point)# what is that? we need two Point_2 here
            cur = cur.parent
        return ret_path


def get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point):
    v = []
    num_of_points_in_dest_direction = random.randint(0, num_of_points_in_batch/5)
    for i in range(num_of_points_in_batch - num_of_points_in_dest_direction):
        coords = [FT(random.uniform(min_x, max_x)) for i in range(2*robot_num)]
        v.append(Point_d(2*robot_num, coords))
    # we should try and steer to goal with some probability
    for i in range(num_of_points_in_dest_direction):
        coords = [(FT(random.uniform(min_x, max_x)) + dest_point[i])/FT(2)for i in range(2*robot_num)]
        v.append(Point_d(2*robot_num, coords))
    return v


def get_min_max(obstacles):
    max_x = max([max([v.x() for v in obs]) for obs in obstacles])
    max_y = max([max([v.y() for v in obs]) for obs in obstacles])
    min_x = min([min([v.x() for v in obs]) for obs in obstacles])
    min_y = min([min([v.y() for v in obs]) for obs in obstacles])
    return max_x.to_double(), max_y.to_double(), min_x.to_double(), min_y.to_double()


def get_square_mid(robot):
    x = (robot[0].x()+robot[1].x())/FT(2)
    y = (robot[1].y()+robot[2].y())/FT(2)
    return [x, y]


def k_nn(tree, k, query, eps):
    search_nearest = True
    sort_neighbors = True
    # TODO: Experiment with a custom distance (i.e. max between the two 2D-Euclidean distances, I feel like that makes more sense)
    search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    lst = []
    search.k_neighbors(lst)
    return lst


def distance_squared(robot_num, p1, p2):
    # return transformed_distance(p1, p2)
    tmp = FT(0)
    for i in range(2*robot_num):
        tmp = tmp + (p1[i] - p2[i]) * (p1[i] - p2[i])
    return tmp


def get_nearest(robot_num, tree, new_points, rand):
    nn = k_nn(tree, 1, rand, FT(0))
    nn_in_tree = nn[0]
    if len(new_points) == 0:
        return nn_in_tree[0]
    # check distance from new points
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
        return Point_d(2*robot_num, [near[i]+(rand[i]-near[i])*eta/dist for i in range(2*robot_num)])


def between(s, p, f):
    return (s <= p <= f) or (f <= p <= s)


def min_dist_between_moving_robots(s1, s2, t1, t2):
    # 2DO Enumerate over all Gods, Godesses, Kami, etc. and for each one pray that this code works.
    sx = s1[0]-s2[0]
    sy = s1[1]-s2[1]

    tx = t1[0]-t2[0]
    ty = t1[1]-t2[1]

    dx = tx - sx
    dy = ty - sy

    m = dy/dx
    n = (dx*sy-dy*sx)/dx
    cands = [max(abs(sx), abs(sy)), max(abs(tx), abs(ty))]
    if m != 1:
        z = n/(1-m)
        if between(sx, z, tx):
            cands.append(abs(z))
    if m != -1:
        z = n/(-1-m)
        if between(sx, z, tx):
            cands.append(abs(z))
    return min(cands)


def paths_too_close(start_point, target_point, robot_num, robot_width):
    # TODO Integrate this instead of the current "many polls" method we use to determine if paths are colliding with eachother.
    for i in range(robot_num):
        s1 = (start_point[2*i].to_double(), start_point[2*i+1].to_double())
        t1 = (target_point[2*i].to_double(), target_point[2*i+1].to_double())
        for j in range(i+1, robot_num):
            s2 = (start_point[2*j].to_double(), start_point[2*j+1].to_double())
            t2 = (target_point[2*j].to_double(), target_point[2*j+1].to_double())
            if min_dist_between_moving_robots(s1, s2, t1, t2) < robot_width.to_double():
                return True
    return False


def is_in_free_face(point_locator, point):
    face = Face()
    # locate can return a vertex or an edge or a face
    located_obj = point_locator.locate(point)
    # TODO Are we sure that we want to return False and not True if we're on a vertex/halfedge? Aren't the robots open, so it should be "True"?
    if located_obj.is_vertex():
        return False
    if located_obj.is_halfedge():
        return False
    if located_obj.is_face():
        located_obj.get_face(face)
    return face.data()[FREESPACE]


def path_collision_free(point_locator, robot_num, p1, p2, robot_width):
    max_robot_path_len = FT(0)
    for j in range(robot_num):
        if not is_in_free_face(point_locator, Point_2(p2[2*j], p2[2*j+1])):
            return False
    for i in range(robot_num):
        robot_path_len = (p2[2*i]-p1[2*i])*(p2[2*i]-p1[2*i])+\
                         (p2[2*i+1]-p1[2*i+1])*(p2[2*i+1]-p1[2*i+1])
        if robot_path_len > max_robot_path_len:
            max_robot_path_len = robot_path_len
    sample_amount = FT(sqrt(max_robot_path_len.to_double()))/inflation_epsilon+FT(1)
    diff_vec = [((p2[i]-p1[i])/sample_amount) for i in range(2*robot_num)]
    curr = [p1[i] for i in range(2*robot_num)]
    for i in range(int(sample_amount.to_double())):
        curr = [sum(x, FT(0)) for x in zip(curr, diff_vec)]
        for j in range(robot_num):
            if not is_in_free_face(point_locator, Point_2(curr[2*j], curr[2*j+1])):
                return False
    return not paths_too_close(p1, p2, robot_num, robot_width)


def try_connect_to_dest(graph, point_locator, robot_num, tree, dest_point, robot_width):
    nn = k_nn(tree, k_nearest, dest_point, FT(0))
    for neighbor in nn:
        if path_collision_free(point_locator, robot_num, neighbor[0], dest_point, robot_width):
            graph[dest_point] = RRT_Node(dest_point, graph[neighbor[0]])
            return True
    return False


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



def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    start = time.time()
    # TODO make sure square is unit square
    robot_width = robots[0][1].x() - robots[0][0].x()
    assert(robot_width == FT(1)) # TODO Instead of having this assert here, make sure we can work even if it's not true (basically, make sure everywhere we assume width to be 1 can work with any width instead.)
    robot_num = len(robots)
    assert(len(destination) == robot_num)
    # init obs for collision detection
    inf_sq_coord = (robot_width+inflation_epsilon)/FT(2)
    v1 = Point_2(inf_sq_coord,          inf_sq_coord)
    v2 = Point_2(inf_sq_coord*FT(-1),   inf_sq_coord)
    v3 = Point_2(inf_sq_coord*FT(-1),   inf_sq_coord*FT(-1))
    v4 = Point_2(inf_sq_coord,          inf_sq_coord*FT(-1))
    inflated_square = Polygon_2([v1, v2, v3, v4])
    cgal_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
    c_space_obstacles = [minkowski_sum_by_full_convolution_2(inflated_square, obs) for obs in cgal_obstacles]
    c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
    single_arrangement = overlay_multiple_arrangements(c_space_arrangements, merge_faces_by_freespace_flag)
    point_locator = Arr_naive_point_location(single_arrangement)

    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    start_ref_points = [get_square_mid(robot) for robot in robots]
    target_ref_points = [[dest.x(), dest.y()] for dest in destination]
    start_point = Point_d(2*robot_num, sum(start_ref_points, []))
    dest_point = Point_d(2*robot_num, sum(target_ref_points, []))
    vertices = [start_point]
    graph = {start_point:RRT_Node(start_point)}
    tree = Kd_tree(vertices)
    while True:
        print("new batch, " + "time= " + str(time.time() - start))
        # I use a batch so that the algorithm can be iterative
        batch = get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point)
        new_points = []
        for p in batch:
            near = get_nearest(robot_num, tree, new_points, p)
            new = steer(robot_num, near, p, steer_eta)
            if path_collision_free(point_locator, robot_num, near, new, robot_width):
                new_points.append(new)
                vertices.append(new)
                graph[new] = RRT_Node(new, graph[near])
            elif use_single_robot_movement:
                for i in range(robot_num):
                    new_data = [near[j] for j in range(2*i-1)]
                    new_data.append(new[2 * i])
                    new_data.append(new[2 * i + 1])
                    new_data = new_data + [near[j] for j in range(2 * i + 2, 2*robot_num)]
                    my_new = Point_d(2*robot_num, new_data)
                    if path_collision_free(point_locator, robot_num, near, my_new, robot_width):
                        new_points.append(my_new)
                        vertices.append(my_new)
                        graph[my_new] = RRT_Node(my_new, graph[near])

        # this in in-efficient if this becomes a bottleneck we should hold an array of kd-trees
        # each double the size of the previous one
        tree.insert(new_points)
        print("vertices amount: "+str(len(vertices)))
        if try_connect_to_dest(graph, point_locator, robot_num, tree, dest_point, robot_width):
            break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    print("finished, " + "time= " + str(time.time() - start))


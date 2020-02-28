from arr2_epec_seg_ex import *
from utils.metrics.distance import distance_squared
from math import log2


def k_nn(tree, k, query, eps):
    search_nearest = True
    sort_neighbors = True
    # print("pre search")
    search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    # print("post search")
    lst = []
    search.k_neighbors(lst)
    return lst


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


def find_neighborhood(robot_num, tree, new_points, sphere_center, eta):
    n = len(new_points) + tree.size()
    r = min(FT(3*(log2(n)/n)**(1/(2*robot_num+1))),
            eta)

    # Circle centered at arg0 with radius of arg1 and epsilon set to arg2
    sphere = Fuzzy_sphere(sphere_center, r, FT(0))

    res = []
    tree.search(sphere, res)
    for point in new_points:
        if distance_squared(robot_num, sphere_center, point) <= r * r:
            res.append(point)
    return res

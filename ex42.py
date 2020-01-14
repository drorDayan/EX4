from arr2_epec_seg_ex import *
import random


def get_batch(num_of_points_in_batch, max_x, max_y, min_x, min_y):
    v = []
    for i in range(num_of_points_in_batch):
        x1 = FT(random.uniform(min_x, max_x))
        y1 = FT(random.uniform(min_y, max_y))
        x2 = FT(random.uniform(min_x, max_x))
        y2 = FT(random.uniform(min_y, max_y))
        v.append(Point_d(4, [x1, y1, x2, y2]))
    return v



def get_min_max(obstacles):
    max_x = max([max([v.x() for v in obs]) for obs in obstacles])
    max_y = max([max([v.y() for v in obs]) for obs in obstacles])
    min_x = min([min([v.x() for v in obs]) for obs in obstacles])
    min_y = min([min([v.y() for v in obs]) for obs in obstacles])
    return FT.to_double(max_x), FT.to_double(max_y), FT.to_double(min_x), FT.to_double(min_y)


def get_square_mid(robot):
    x = (robot[0].x()+robot[1].x())/FT(2)
    y = (robot[1].y()+robot[2].y())/FT(2)
    return x, y


def k_nn(tree, k, query, eps):
    search_nearest = True
    sort_neighbors = True
    # search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    lst = []
    search.k_neighbors(lst)
    return lst


def distance(p1, p2):
    tmp = FT(0)
    for i in range(4):
        tmp = tmp + (p1[i] - p2[i]) * (p1[i] - p2[i])
    return tmp


def get_nearest(tree, new_points, rand):
    nn = k_nn(tree, 1, rand, FT(0))
    nn_in_tree = nn[0]
    if len(new_points) == 0:
        return nn_in_tree[0]
    # check distance from new points
    dist = [distance(rand, point) for point in new_points]
    min_dist = dist[0]
    min_i = 0
    for i in range(len(new_points)):
        if dist[i] < min_dist:
            min_dist = dist[i]
            min_i = i
    if min_dist < nn_in_tree[1] * nn_in_tree[1]:
        return new_points[min_i]
    return nn_in_tree[0]


def steer(near, rand, eta):
    # TODO we should try and steer to goal with some probability
    return Point_d(4, [near[i]+rand[i]*eta/FT(4) for i in range(4)])


def collision_free(p1, p2):
    # TODO implement
    return True


def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    # TODO init obs for collision detection
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    num_of_points_in_batch = 200
    eta = FT(0.5)
    r1x, r1y = get_square_mid(robots[0])
    r2x, r2y = get_square_mid(robots[1])
    start_point = Point_d(4, [r1x, r1y, r2x, r2y])
    vertices = [start_point]
    edges = []
    print(vertices)
    print(edges)
    batch = get_batch(num_of_points_in_batch, max_x, max_y, min_x, min_y)
    tree = Kd_tree(vertices)
    new_points = []
    for p in batch:
        near = get_nearest(tree, new_points, p)
        print(near)
        new = steer(near, p, eta)
        print(new)
        if collision_free(near, new):
            new_points.append(new)
            vertices.append(new)
            edges.append((near, new))
    print(vertices)
    print(edges)


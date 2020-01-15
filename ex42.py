from arr2_epec_seg_ex import *
import random
import time


def get_batch(num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point):
    v = []
    num_of_points_in_dest_direction = random.randint(0, num_of_points_in_batch/5)
    for i in range(num_of_points_in_batch - num_of_points_in_dest_direction):
        x1 = FT(random.uniform(min_x, max_x))
        y1 = FT(random.uniform(min_y, max_y))
        x2 = FT(random.uniform(min_x, max_x))
        y2 = FT(random.uniform(min_y, max_y))
        v.append(Point_d(4, [x1, y1, x2, y2]))
    # we should try and steer to goal with some probability
    for i in range(num_of_points_in_dest_direction):
        x1 = (FT(random.uniform(min_x, max_x)) + dest_point[0])/FT(2)
        y1 = (FT(random.uniform(min_y, max_y)) + dest_point[1])/FT(2)
        x2 = (FT(random.uniform(min_x, max_x)) + dest_point[2])/FT(2)
        y2 = (FT(random.uniform(min_y, max_y)) + dest_point[3])/FT(2)
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
    # TODO need to handle rand points closer than eta
    return Point_d(4, [near[i]+rand[i]*eta/FT(4) for i in range(4)])


def collision_free(p1, p2):
    # TODO implement
    return True


def try_connect_to_dest(tree, dest_point):
    nn = k_nn(tree, 10, dest_point, FT(0))
    for neighbor in nn:
        if collision_free(neighbor[0], dest_point):
            # TODO add edge
            return True
    return False


def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    start = time.time()
    # TODO init obs for collision detection
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    num_of_points_in_batch = 200
    eta = FT(0.5)
    r1x, r1y = get_square_mid(robots[0])
    r2x, r2y = get_square_mid(robots[1])
    start_point = Point_d(4, [r1x, r1y, r2x, r2y])
    dest_point = Point_d(4, [destination[0].x(), destination[0].y(), destination[1].x(), destination[1].y()])
    vertices = [start_point]
    edges = []
    print(vertices)
    print(edges)
    tree = Kd_tree(vertices)
    while True:
        print("new batch, " + "time= " + str(time.time() - start))
        # I use a batch so that the algorithm can be iterative
        batch = get_batch(num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point)
        new_points = []
        for p in batch:
            near = get_nearest(tree, new_points, p)
            print(near)
            new = steer(near, p, eta)
            print(new)
            if collision_free(near, new):
                new_points.append(new)
                vertices.append(new)
                # TODO this is not a good way to hold edges should change it after we understand collision detection
                edges.append((near, new))
        # this in in-efficient if this becomes a bottleneck we should hold an array of kd-trees
        # each double the size of the previous one
        tree.insert(new_points)
        if try_connect_to_dest(tree, dest_point):
            break
    # TODO create the result (it is possible if we reached this point) use previous exercise bfs
    print(vertices)
    print(edges)
    print("finished, " + "time= " + str(time.time() - start))


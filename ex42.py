from arr2_epec_seg_ex import *
import random
import time

# Configurable Variables: #

k_nearest = 10
steer_eta = FT(0.5)
inflation_epsilon = FT(0.5)

def get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point):
    v = []
    num_of_points_in_dest_direction = random.randint(0, num_of_points_in_batch/5)
    for i in range(num_of_points_in_batch - num_of_points_in_dest_direction):
        # x1 = FT(random.uniform(min_x, max_x))
        # y1 = FT(random.uniform(min_y, max_y))
        # x2 = FT(random.uniform(min_x, max_x))
        # y2 = FT(random.uniform(min_y, max_y))
        coords = [FT(random.uniform(min_x, max_x)) for i in range(2*robot_num)]
        v.append(Point_d(2*robot_num, coords))
    # we should try and steer to goal with some probability
    for i in range(num_of_points_in_dest_direction):
        # x1 = (FT(random.uniform(min_x, max_x)) + dest_point[0])/FT(2)
        # y1 = (FT(random.uniform(min_y, max_y)) + dest_point[1])/FT(2)
        # x2 = (FT(random.uniform(min_x, max_x)) + dest_point[2])/FT(2)
        # y2 = (FT(random.uniform(min_y, max_y)) + dest_point[3])/FT(2)
        coords = [(FT(random.uniform(min_x, max_x)) + dest_point[i])/FT(2)for i in range(2*robot_num)]
        v.append(Point_d(2*robot_num, coords))
    return v


def get_min_max(obstacles):
    max_x = max([max([v.x() for v in obs]) for obs in obstacles])
    max_y = max([max([v.y() for v in obs]) for obs in obstacles])
    min_x = min([min([v.x() for v in obs]) for obs in obstacles])
    min_y = min([min([v.y() for v in obs]) for obs in obstacles])
    return FT.to_double(max_x), FT.to_double(max_y), FT.to_double(min_x), FT.to_double(min_y)


def get_square_mid(robot):
    x = (robot[0].x()+robot[1].x())/FT(2)
    # TODO: (dror) aren't those supposed to be 0,1 instead of 1,2 (the line below)
    y = (robot[1].y()+robot[2].y())/FT(2)
    return [x, y]


def k_nn(tree, k, query, eps):
    search_nearest = True
    sort_neighbors = True
    # search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    # TODO: Experiment with a custom distance (i.e. max between the two 2D-Euclidean distances, I feel like that makes more sense)
    search = K_neighbor_search(tree, query, k, eps, search_nearest, Euclidean_distance(), sort_neighbors)
    lst = []
    search.k_neighbors(lst)
    return lst


def distance_squared(robot_num, p1, p2):
    tmp = FT(0)
    for i in range(2*robot_num):
        tmp = tmp + (p1[i] - p2[i]) ** 2
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
    dist = distance_squared(robot_num, near, rand) ** 0.5
    if dist < eta:
        return rand
    else:
        return Point_d(2*robot_num, [near[i]+(rand[i]-near[i])*eta/dist for i in range(2*robot_num)])


def between(s, p, f):
    return (s <= p <= f) or (f <= p <= s)


def min_dist_between_moving_robots(s1, s2, t1, t2):
    # TODO Enumerate over all Gods, Godesses, Kami, etc. and for each one pray that this code works.
    sx = s1[0]-s2[0]
    sy = s1[1]-s2[1]

    dy = t1[1]-t2[1]-sy
    dx = t1[0]-t2[0]-sx

    m = dy/dx
    n = (dx*sy-dy*sx)/dx
    cands = [max(s[0], s[1]), max(d[0], d[1])]
    if m != 1:
        t = n/(1-m)
        if between(sx, t, sx+dx):
            cands.append(t)
    if m != -1:
        t = n/(-1-m)
        if between(sx, t, sx+dx):
            cands.append(t)
    return min(cands)


def paths_too_close(start_point,target_point, robot_width):
    # TODO turn start_point, target_point, which are of type Point_d, into s1,s2,t1,t2 that need to be tuples of 2 floats each.
    #      s1,s2 are the start coordinates of two robots, t1,t2 are the target coordinates
    # TODO support more than 2 robots?
    return min_dist_between_moving_robots(s1, s2, t1, t2) < robot_width


def collision_free(robot_num, p1, p2):
    # TODO implement
    return True


def try_connect_to_dest(robot_num, tree, dest_point):
    nn = k_nn(tree, k_nearest, dest_point, FT(0))
    for neighbor in nn:
        if collision_free(robot_num, neighbor[0], dest_point):
            # TODO add edge
            return True
    return False


def generate_path(path, robots, obstacles, destination):
    print(path, robots, obstacles, destination)
    robot_num = len(robots)
    assert(len(destination) == robot_num)
    start = time.time()
    # TODO init obs for collision detection
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    num_of_points_in_batch = 200
    # eta = FT(0.5)
    start_ref_points = [get_square_mid(robot) for robot in robots]
    target_ref_points = [get_square_mid(dest) for dest in destination]
    #r1x, r1y = get_square_mid(robots[0])
    #r2x, r2y = get_square_mid(robots[1])
    start_point = Point_d(2*robot_num, sum(start_ref_points,[]))
    dest_point = Point_d(2*robot_num, sum(target_ref_points,[]))
    vertices = [start_point]
    edges = []
    print(vertices)
    print(edges)
    tree = Kd_tree(vertices)
    while True:
        print("new batch, " + "time= " + str(time.time() - start))
        # I use a batch so that the algorithm can be iterative
        batch = get_batch(robot_num, num_of_points_in_batch, max_x, max_y, min_x, min_y, dest_point)
        new_points = []
        for p in batch:
            near = get_nearest(robot_num, tree, new_points, p)
            print(near)
            new = steer(robot_num, near, p, steer_eta)
            print(new)
            if collision_free(robot_num, near, new):
                new_points.append(new)
                vertices.append(new)
                # TODO this is not a good way to hold edges should change it after we understand collision detection
                edges.append((near, new))
        # this in in-efficient if this becomes a bottleneck we should hold an array of kd-trees
        # each double the size of the previous one
        tree.insert(new_points)
        if try_connect_to_dest(robot_num, tree, dest_point):
            break
    # TODO create the result (it is possible if we reached this point) use previous exercise bfs
    print(vertices)
    print(edges)
    print("finished, " + "time= " + str(time.time() - start))


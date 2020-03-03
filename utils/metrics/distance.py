from arr2_epec_seg_ex import Euclidean_distance, Point_d


def distance_squared(p1, p2):
    return Euclidean_distance().transformed_distance(p1, p2)


def distances_squared(p1, p2):
    assert (p1.dimension() == p2.dimension())
    retu = [distance_squared(Point_d(2, [p1[2 * i], p1[2 * i + 1]]),
                             Point_d(2, [p2[2 * i], p2[2 * i + 1]]))
            for i in range(int(p1.dimension()/2))]
    return retu

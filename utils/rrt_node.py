from utils.clearance import *
from utils.distance import *


class RRT_Cost(object):
    def __init__(self, weight, calc_segment, reduce_path):
        self.weight = weight
        self.calc_segment = calc_segment
        self.reduce_path = reduce_path


class RRT_Node(object):
    costs = {
        "distance": RRT_Cost(1,
                             lambda current, parent: distance_squared(current.robot_num,
                                                                      parent.point,
                                                                      current.point),
                             lambda my_val, parent_val: my_val + parent_val
                             ),
        "inv_clearance_o": RRT_Cost(0,
                                    lambda current, parent: FT(1) / calc_clearance_with_obstacles(current.robot_num,
                                                                                                  parent.point,
                                                                                                  current.point),
                                    lambda old, update: min(old, update)
                                    ),
        "inv_clearance_r": RRT_Cost(1,
                                    lambda current, parent: FT(1) / min_inter_robot_distance(current.robot_num,
                                                                                             parent.point,
                                                                                             current.point),
                                    lambda old, update: min(old, update)
                                    )
    }

    def __init__(self, pt, robot_num, pr=None):
        self.point = pt
        self.parent = pr

        self.robot_num = robot_num
        self.metrics = {}
        self.calc_metrics()

    def calc_metrics(self):
        pr = self.parent
        if pr is not None:
            for k in self.costs.keys():
                if self.costs[k].weight > 0:
                    self.metrics[k] = self.costs[k].calc_segment(self, pr)
                else:
                    self.metrics[k] = FT(0)
        else:
            for k in self.costs.keys():
                self.metrics[k] = FT(0)

    def calc_path_metrics(self):
        my_values = {}
        if self.parent is None:
            for metric in self.costs.keys():
                if self.costs[metric].weight > 0:
                    my_values[metric] = None
        else:
            parent_values = self.parent.calc_path_metrics()
            for metric in parent_values.keys():
                if parent_values[metric] is None:
                    my_values[metric] = self.metrics[metric]
                else:
                    my_values[metric] = self.costs[metric].reduce_path(self.metrics[metric],
                                                                       parent_values[metric])
        return my_values

    def calc_value(self):
        separate_values = self.calc_path_metrics()
        print(separate_values)
        return sum([separate_values[metric] * FT(self.costs[metric].weight) for metric in separate_values.keys()],
                   FT(0))

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path
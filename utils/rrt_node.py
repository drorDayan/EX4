from utils.metrics.clearance import *
from utils.metrics.distance import *
from config import *


class RRT_Cost(object):
    def __init__(self, weight, calc_segment, reduce_path):
        self.weight = weight
        self.calc_segment = calc_segment
        self.reduce_path = reduce_path


class RRT_Node(object):
    costs = {
        "distance": RRT_Cost(weight_distance,
                             lambda current, parent: distance_squared(current.robot_num,
                                                                      parent.point,
                                                                      current.point),
                             lambda my_val, parent_val: my_val + parent_val
                             ),
        "inv_clearance_o": RRT_Cost(weight_obstacle_clearance_inv,
                                    lambda current, parent: FT(1) / calc_clearance_with_obstacles(current.robot_num,
                                                                                                  parent.point,
                                                                                                  current.point),
                                    lambda old, update: max(old, update)
                                    ),
        "inv_clearance_r": RRT_Cost(weight_robot_clearance_inv,
                                    lambda current, parent: FT(1) / min_inter_robot_distance(current.robot_num,
                                                                                             parent.point,
                                                                                             current.point),
                                    lambda old, update: max(old, update)
                                    )
    }

    def __init__(self, pt, robot_num, pr=None):
        self.point = pt
        self.parent = pr

        self.robot_num = robot_num
        self.has_metric = False
        for metric in self.costs.keys():
            if self.costs[metric].weight > 0:
                self.has_metric = True
                break
        if self.has_metric:
            self.values = self.segment_to_target_values_dict(self.parent)

    def set_parent(self, new_parent):
        self.parent = new_parent
        if self.has_metric:
            self.values = self.segment_to_target_values_dict(new_parent)

    def segment_to_target_values_dict(self, target):
        if not self.has_metric:
            return {}
        metrics = {}
        if target is not None:
            for k in self.costs.keys():
                if self.costs[k].weight > 0:
                    metrics[k] = self.costs[k].calc_segment(self, target)
                else:
                    metrics[k] = FT(0)
        else:
            for k in self.costs.keys():
                metrics[k] = FT(0)
        return metrics

    def path_to_origin_values_dict(self):
        if not self.has_metric:
            return {}
        final_values = {}
        if self.parent is None:
            for metric in self.costs.keys():
                if self.costs[metric].weight > 0:
                    final_values[metric] = None
        else:
            parent_values = self.parent.path_to_origin_values_dict()
            for metric in parent_values.keys():
                if parent_values[metric] is None:
                    final_values[metric] = self.values[metric]
                else:
                    final_values[metric] = self.costs[metric].reduce_path(self.values[metric],
                                                                          parent_values[metric])
        return final_values

    def path_to_origin_value(self):
        if not self.has_metric:
            return 0
        return self.path_to_origin_through_target_values(self.parent)[0]

    def path_to_origin_through_target_values(self, target):
        if not self.has_metric:
            return FT(0), FT(0)
        if target is None:
            return FT(0), FT(0)

        first_segment_separate_values = self.segment_to_target_values_dict(target)
        target_separate_values = target.path_to_origin_values_dict()

        my_separate_values = {}
        for metric in target_separate_values.keys():
            if target_separate_values[metric] is None:
                my_separate_values[metric] = first_segment_separate_values[metric]
                target_separate_values[metric] = FT(0)
            else:
                my_separate_values[metric] = self.costs[metric].reduce_path(first_segment_separate_values[metric],
                                                                            target_separate_values[metric])

        return [sum([values_dict[metric] * FT(self.costs[metric].weight) for metric in values_dict.keys()],
                    FT(0)) for values_dict in (my_separate_values, target_separate_values)]

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path

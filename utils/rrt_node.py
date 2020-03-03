from utils.metrics.robot_clearance import *
from utils.metrics.turn_angle import *
from utils.metrics.distance import *
from config import *


class RRT_Cost(object):
    def __init__(self, weight, calc_segment, reduce_path, extract_value=lambda x: x):
        self.weight = weight
        self.calc_segment = calc_segment
        self.reduce_path = reduce_path
        self.extract_value = extract_value


class RRT_Node(object):
    costs = {
        "distances_sum": RRT_Cost(weight_distance_sum,
                                  lambda current, parent: distances_squared(parent.point, current.point),
                                  lambda my_val, parent_val: [my_val[i] + parent_val[i] for i in range(len(my_val))],
                                  lambda lengths_list: sum(lengths_list, FT(0))),
        "distances_max": RRT_Cost(weight_distance_max,
                                  lambda current, parent: distances_squared(parent.point, current.point),
                                  lambda my_val, parent_val: [my_val[i] + parent_val[i] for i in range(len(my_val))],
                                  lambda lengths_list: max(lengths_list)),
        # "distance": RRT_Cost(weight_distance_sum,
        #                      lambda current, parent: distance_squared(parent.point, current.point),
        #                      lambda my_val, parent_val: my_val + parent_val
        #                      ),
        "inv_clearance_r": RRT_Cost(weight_robot_clearance_inv,
                                    lambda current, parent: FT(1) / min_inter_robot_distance(parent.point,
                                                                                             current.point),
                                    lambda my_val, parent_val: max(my_val, parent_val)
                                    ),
        "turn_angle": RRT_Cost(weight_turn_angle,
                               lambda current, parent: calc_angle_transformed(None if parent.parent is None else
                                                                              parent.parent.point,
                                                                              parent.point, current.point),
                               lambda my_val, parent_val: max(my_val, parent_val)
                               )
    }

    def __init__(self, pt, pr=None):
        self.point = pt
        self.parent = pr

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

    def reduce_dict_to_value(self, values_dict):
        return sum([values_dict[metric] * FT(self.costs[metric].weight) for metric in values_dict.keys()], FT(0))

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
                target_separate_values[metric] = self.costs[metric].extract_value(target_separate_values[metric])
            my_separate_values[metric] = self.costs[metric].extract_value(my_separate_values[metric])
        return [self.reduce_dict_to_value(values_dict) for values_dict in (my_separate_values, target_separate_values)]

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path

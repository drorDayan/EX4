from arr2_epec_seg_ex import *
import math
import importlib
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import read_input
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list
import ms_polygon_segment, linear_path_intersection_test

offset = -Vector_2(FT(0.5), FT(0.5))

class Polygons_scene():
  def __init__(self):
    self.robot_num = 0
    self.robots = []
    self.obstacles = []
    self.gui_robots = []
    self.gui_obstacles = []
    self.destinations = []
    self.gui_destinations = []
    self.path = []

  def draw_scene(self):
    gui.clear_scene()
    colors = [Qt.yellow, Qt.green, Qt.black, Qt.blue, Qt.darkGreen, Qt.red, Qt.magenta, Qt.darkMagenta, Qt.darkGreen, Qt.darkCyan, Qt.cyan]
    for i in range(len(self.robots)):
      if (self.robots[i] != None):
        self.gui_robots[i] = gui.add_polygon([point_2_to_xy(p) for p in self.robots[i]], colors[i])
    for obstacle in self.obstacles:
      self.gui_obstacles = []
      self.gui_obstacles.append(gui.add_polygon([point_2_to_xy(p) for p in obstacle], Qt.darkGray))
    for i in range(len(self.destinations)):
      if(self.destinations[i] != None):
        self.gui_destinations[i] = gui.add_disc(0.05, *point_2_to_xy(self.destinations[i]), colors[i])

  def load_scene(self, filename):
    scene = read_input.read_polygon_scene(filename)
    self.robot_num = scene[0]
    # gui.set_field(2, " ".join(scene[0]))
    destinations = [None for i in range(self.robot_num)]
    self.gui_destinations = [None for i in range(self.robot_num)]
    s = []
    for i in range(self.robot_num):
      destinations[i] = scene[i+1]
      # s.append(str(destinations[i].x().exact()) + " " + str(destinations[i].y().exact()))
      # gui.set_field(i+5, s[i])
    self.set_destinations(destinations)
    self.robots = [None for i in range(self.robot_num)]
    self.gui_robots = [None for i in range(self.robot_num)]
    for i in range(self.robot_num):
      self.robots[i] = scene[i+self.robot_num+1]
    self.obstacles = []
    for i in range(1+2*self.robot_num, len(scene)):
      self.obstacles.append(scene[i])
    gui.clear_queue()
    self.draw_scene()

  def set_destinations(self, destinations):
    self.destinations = destinations
    for i in range(len(self.gui_destinations)):
      if self.gui_destinations[i] == None:
        self.gui_destinations[i] = gui.add_disc(0.05, *point_2_to_xy(destinations[i]), Qt.green)
      else:
        self.gui_destinations[i].pos = QPointF(*point_2_to_xy(destinations[i]))

  def set_up_animation(self):
    self.draw_scene()
    colors = [Qt.yellow, Qt.green, Qt.black, Qt.blue, Qt.darkGreen, Qt.red, Qt.magenta, Qt.darkMagenta, Qt.darkGreen,
              Qt.darkCyan, Qt.cyan]
    if len(self.path) == 0:
      return
    if len(self.path) == 1:
      return
    else:
      for i in range(len(self.path) - 1):
        anim_l = [None for i in range(self.robot_num)]
        for r_i in range(self.robot_num):
          start = point_2_to_xy(self.path[i][r_i])
          end = point_2_to_xy(self.path[i+1][r_i])
          s = gui.add_segment(*start, *end, colors[r_i])
          start = point_2_to_xy(self.path[i][r_i] + offset)
          end = point_2_to_xy(self.path[i + 1][r_i] + offset)
          s.line.setZValue(2)
          anim_l[r_i] = gui.linear_translation_animation(self.gui_robots[r_i], *start, *end)
        anim = gui.parallel_animation(anim_l)
        gui.queue_animation(anim)

  def is_path_valid(self):
    check22 = True
    if self.path == None: return False
    if len(self.path) == 0: return False
    robot_polygons = [None for i in range(self.robot_num)]
    path_polygons = []
    if len(self.path) > 1:
      for i in range(len(self.path) - 1):
        source = [None for i in range(self.robot_num)]
        target = [None for i in range(self.robot_num)]
        for j in range(len(self.robots)):
          robot_polygons[j] = Polygon_2(self.robots[j])
          source[j] = self.path[i][j]
          target[j] = self.path[i+1][j]
          if source[j] != target[j]:
            s = Segment_2(source[j], target[j])
            pwh = ms_polygon_segment.minkowski_sum_polygon_segment(robot_polygons[j], s)
          else:
            pwh = ms_polygon_segment.minkowski_sum_polygon_point(robot_polygons[j], source[j])
          path_polygons.append(pwh)
        for j in range(len(self.robots)):
          for k in range(j, self.robot_num):
            if linear_path_intersection_test.do_intersect(robot_polygons[j], robot_polygons[k], source, target): check22 = False

    obstacle_polygons = []
    for obs in self.obstacles:
      p = Polygon_2(obs)
      obstacle_polygons.append(p)

    path_set = Polygon_set_2()
    path_set.join_polygons_with_holes(path_polygons)
    obstacles_set = Polygon_set_2()
    obstacles_set.join_polygons(obstacle_polygons)

    lst = []
    path_set.polygons_with_holes(lst)
    for pwh in lst:
      p = pwh.outer_boundary()
      lst = polygon_2_to_tuples_list(p)
      gui.add_polygon(lst, Qt.lightGray).polygon.setZValue(-3)
      for p in pwh.holes():
        lst = polygon_2_to_tuples_list(p)
        gui.add_polygon(lst, Qt.white).polygon.setZValue(-2)


    # check that the origin matches the first point in the path
    check01 = True
    for i in range(self.robot_num):
      if self.robots[i][0] - offset != self.path[0][i]:
        check01 = False
    # check that the destination matches the last point in the path
    check11 = True
    for i in range(self.robot_num):
      if self.destinations[i] != self.path[-1][i]:
        check11 = False
    #check that there are no collisions
    check21 = True if not path_set.do_intersect(obstacles_set) else False
    res = (check01 and check11 and check21 and check22)
    print("Valid path: ", res)
    if check01 == False :
      print("Origin mismatch")
      print(self.robots[0][0] - offset, self.robots[1][0] - offset)
      print(self.path[0][0], self.path[0][1])
    if check11 == False :
      print("Destination mismatch")
      print(self.destinations[0], self.destinations[1])
      print(self.path[-1][0], self.path[-1][1])
    if check21 == False:
      print("Movement along path intersects with obstacles")
    if check22 == False:
      print("The robots intersect each other")
    return res

def set_up_scene():
  gui.clear_scene()
  # ps.destinations = [None, None]
  # ps.gui_destinations = [None, None]
  scene_file = gui.get_field(0)
  ps.load_scene(scene_file)
  print("loaded scene from", scene_file)

def generate_path():
  ps.path = []
  gui.clear_queue()
  path_name = gui.get_field(3)
  gp = importlib.import_module(path_name)
  #for steer_eta in [FT(0.1), FT(0.2), FT(0.3), FT(0.4), FT(0.5), FT(0.6), FT(0.7), FT(0.8), FT(0.9), FT(1.0)]:
  #  for i in range(10):
  gp.generate_path(ps.path, ps.robots, ps.obstacles, ps.destinations)
  # print("Generated path via", path_name + ".generate_path")
  ps.set_up_animation()

def load_path():
  ps.path = []
  gui.clear_queue()
  path_name = gui.get_field(4)
  read_input.load_path(ps.path, path_name)
  print("Loaded path from", path_name)
  ps.set_up_animation()

# def save_path():
#   path_name = gui.get_field(5)
#   with open(path_name, 'w') as f:
#     for i in range(len(ps.path)):
#       p = ps.path[i]
#       x = p.x().exact()
#       y = p.y().exact()
#       f.write(str(x.numerator()) + "/" + str(x.denominator()) + " " + str(y.numerator()) + "/" + str(y.denominator()))
#       if i != len(ps.path)-1: f.write('\n')
#   print("Path saved to", path_name)

def is_path_valid():
  ps.is_path_valid()

def set_destinations():
  s0 = gui.get_field(1).split(" ")
  s1 = gui.get_field(2).split(" ")
  destinations = [None, None]
  destinations[0] = Point_2(FT(Gmpq(s0[0])), FT(Gmpq(s0[1])))
  destinations[1] = Point_2(FT(Gmpq(s1[0])), FT(Gmpq(s1[1])))
  ps.set_destinations(destinations)

def animate_path():
  gui.play_queue()

if __name__ == "__main__":
  import sys
  app = QtWidgets.QApplication(sys.argv)
  gui = GUI()
  ps = Polygons_scene()
  gui.set_program_name("Multi-robot Motion Planning")
  gui.set_field(0, "scene3_very_hard_multi")
  gui.set_field(3, "ex42")
  gui.set_field(4, "path0.txt")
  #gui.set_field(5, "path_out.txt")
  gui.set_logic(0, set_up_scene)
  gui.set_button_text(0, "Load scene")
  gui.set_button_text(1, "unused")
  gui.set_logic(2 , set_destinations)
  gui.set_button_text(2, "Set destinations")
  gui.set_logic(3, generate_path)
  gui.set_button_text(3, "Generate path")
  gui.set_logic(4, load_path)
  gui.set_button_text(4, "Load path")
  #gui.set_logic(5, save_path)
  gui.set_button_text(5, "unused")
  gui.set_logic(6, animate_path)
  gui.set_button_text(6, "Animate movement along path")
  gui.set_logic(7, is_path_valid)
  gui.set_button_text(7, "Check path validity")
  gui.MainWindow.show()
  sys.exit(app.exec_())
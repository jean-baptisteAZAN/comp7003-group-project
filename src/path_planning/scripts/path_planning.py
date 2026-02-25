#!/usr/bin/env python3
from __future__ import annotations
import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from algorithms.neighbors import find_neighbors
import math


class Cell:
    def __init__(self, pos: int, g: int, f: int, parent: Cell):
        self.pos = pos
        self.g = g
        self.f = f
        self.parent = parent

    def update_f(self, new_g, new_h):
        self.f = new_g + new_h


def euclidean_dist(start, goal, width):
    start_x = start % width
    start_y = start // width
    goal_x = goal % width
    goal_y = goal // width
    math.sqrt((start_x - goal_x) ** 2 + (start_y - goal_y) ** 2)


def find_lowest_f(array: list[Cell]):
    minimum = array[0].f

    for i in range(len(array)):
        if array[i].f < minimum:
            minimum = array[i].f

    return minimum


def a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation):
    start_cell = Cell(start, 0, 0, None)
    start_cell.update_f(0, euclidean_dist(0, goal, width))
    to_visit.append(start_cell)

    to_visit = []  # array of cells not position
    visited = []

    while to_visit:
        current_index = find_lowest_f(to_visit)
        current = array[current_index]
        array.pop(current_index)
        visited.append(current)

        if current.pos == goal:
            return  # call function to get path by using the parent cells

        all_neighbors = find_neighbors(
            current.pos, width, height, costmap, 1
        )  # not sure about the last param

        for neighbor in all_neighbors:
            # check if neighbor in visited, if yes, skip (continue)
            in_visited = 0
            for cell in visited:
                if neighbor[0] == cell.pos:
                    in_visited = 1

            if in_visited:
                continue

            # check if neighbor already in to_visit, if yes, check if g_score is smaller, if yes, update cell
            # if not, create new Cell corresponding to neighbor and add it to to_visit
            new_g_score = current.g + neighbor[1]

            in_to_visit = 0
            neighbor_cell: Cell = None
            for i in range(len(to_visit)):
                if neighbor[0] == to_visit[i].pos:
                    in_to_visit = 1
                    neighbor_cell = to_visit[i]

            if in_to_visit:
                if new_g_score < neighbor_cell.g:
                    neighbor_cell.g = new_g_score
            else:
                neighbor_cell = Cell(neighbor[0], new_g_score, 0, current)


def make_plan(req):
    """
    Callback function used by the service server to process
    requests from clients. It returns a msg of type PathPlanningPluginResponse
    """
    costmap = req.costmap_ros
    # number of columns in the occupancy grid
    width = req.width
    # number of rows in the occupancy grid
    height = req.height
    start = req.start
    goal = req.goal
    # side of each grid map square in meters
    resolution = 0.05
    # origin of grid map
    origin = []  # hint: find this in your YAML map file

    grid_visualisation = GridViz(costmap, resolution, origin, start, goal, width)

    # time statistics
    start_time = rospy.Time.now()

    # calculate the shortest path

    """
  Your code continues here.
  path = a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation)

  """
    path = a_star(
        start, goal, width, height, costmap, resolution, origin, grid_visualisation
    )

    if not path:
        rospy.logwarn("No path returned by the path algorithm")
        path = []
    else:
        # additional code here as per your implementation, e.g., computing/displaying your performance metrics
        rospy.loginfo("Path sent to navigation stack")

    resp = PathPlanningPluginResponse()
    resp.plan = path
    return resp


def clean_shutdown():
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("path_planning_server", log_level=rospy.INFO, anonymous=False)
    make_plan_service = rospy.Service(
        "/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan
    )
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    rospy.on_shutdown(clean_shutdown)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2), rospy.signal_shutdown("Shutting down"), oneshot=True)

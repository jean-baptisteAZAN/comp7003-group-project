from __future__ import annotations
import math
from algorithms.neighbors import find_neighbors
import rospy


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
    rospy.loginfo("In Astar")
    start_cell = Cell(start, 0, 0, None)
    start_cell.update_f(0, euclidean_dist(0, goal, width))
    to_visit = [start_cell]  # array of cells not position
    visited = []

    while to_visit:
        current_index = find_lowest_f(to_visit)
        current = to_visit[current_index]
        to_visit.pop(current_index)
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

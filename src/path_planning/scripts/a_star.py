from __future__ import annotations
import math
from algorithms.neighbors import find_neighbors
import rospy


class Cell:
    def __init__(self, pos: int, g, f, parent: Cell):
        self.pos = pos
        self.g = g
        self.f = f
        self.parent = parent


def euclidean_dist(start, goal, width):
    start_x = start % width
    start_y = start // width
    goal_x = goal % width
    goal_y = goal // width
    return math.sqrt((start_x - goal_x) ** 2 + (start_y - goal_y) ** 2)


def find_lowest_f(array: list[Cell]):
    minimum = array[0].f
    min_index = 0

    for i in range(len(array)):
        if array[i].f < minimum:
            minimum = array[i].f
            min_index = i

    return min_index


def get_path(last_cell):
    path = []
    path.append(last_cell.pos)

    while last_cell.parent is not None:
        last_cell = last_cell.parent
        path.append(last_cell.pos)

    path.reverse()
    rospy.loginfo(path)
    return path


def a_star(start, goal, width, height, costmap, resolution, origin, grid_visualisation):
    rospy.loginfo("In Astar")
    rospy.loginfo(f"{start=}")
    rospy.loginfo(f"{resolution=}")
    rospy.loginfo(f"{origin=}")
    rospy.loginfo(f"{goal=}")
    rospy.loginfo(f"{width=}")
    rospy.loginfo(f"{height=}")
    rospy.loginfo(f"{len(costmap)=}")
    rospy.loginfo(f"{len([cell for cell in costmap if cell <= 150])=}")
    start_cell = Cell(start, 0, euclidean_dist(start, goal, width), None)
    to_visit = [start_cell]  # array of cells not position
    visited = set()

    while to_visit:
        current_index = find_lowest_f(to_visit)
        current = to_visit.pop(current_index)
        visited.add(current.pos)
        grid_visualisation.set_color(current.pos, "pale yellow")

        if current.pos == goal:
            return get_path(
                current
            )  # call function to get path by using the parent cells

        all_neighbors = find_neighbors(
            current.pos, width, height, costmap, 1
        )  # 1 for the last param because otherwise, the penalty related to the costmap would be too impactful

        for neighbor in all_neighbors:
            # check if neighbor in visited, if yes, skip (continue)
            if neighbor[0] in visited:
                continue

            # check if neighbor already in to_visit, if yes, check if g_score is smaller, if yes, update cell
            # if not, create new Cell corresponding to neighbor and add it to to_visit
            new_g_score = current.g + neighbor[1]
            new_f_score = new_g_score + euclidean_dist(neighbor[0], goal, width)

            in_to_visit = 0
            neighbor_cell: Cell = None
            for i in range(len(to_visit)):
                if neighbor[0] == to_visit[i].pos:
                    in_to_visit = 1
                    neighbor_cell = to_visit[i]

            if in_to_visit:
                if new_g_score < neighbor_cell.g:
                    neighbor_cell.g = new_g_score
                    neighbor_cell.f = new_f_score
                    neighbor_cell.parent = current
            else:
                grid_visualisation.set_color(neighbor[0], "orange")
                neighbor_cell = Cell(neighbor[0], new_g_score, new_f_score, current)
                to_visit.append(neighbor_cell)

    rospy.loginfo("END")
    rospy.loginfo(f"{len(to_visit)=}")
    rospy.loginfo(f"{len(visited)=}")

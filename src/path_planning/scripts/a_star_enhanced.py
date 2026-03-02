from __future__ import annotations
import math
from algorithms.neighbors import find_neighbors
import rospy
from gridviz import GridViz
from a_star import *


def calculate_angle(previous : Cell, current : Cell, neighbor : Cell, width):
    previous_x, previous_y = previous % width, previous / width
    current_x, current_y = current % width, current / width
    neighbor_x, neighbor_y = neighbor % width, neighbor / width

    ux, uy = previous_x - current_x, previous_y - current_y
    vx, vy = neighbor_x - current_x, neighbor_y - current_y

    dot = ux*vx + uy*vy
    
    norme_u = math.sqrt(ux ** 2 + uy ** 2)
    norme_v = math.sqrt(vx ** 2 + vy ** 2)

    ang = math.acos(dot / (norme_u * norme_v))
    return ang

def a_star_enhanced(start, goal, width, height, costmap, resolution, origin, grid_visualisation):
    rospy.loginfo("In Astar")
    rospy.loginfo(f"{start=}")
    rospy.loginfo(f"{resolution=}")
    rospy.loginfo(f"{origin=}")
    rospy.loginfo(f"{goal=}")
    rospy.loginfo(f"{width=}")
    rospy.loginfo(f"{height=}")
    rospy.loginfo(f"{len(costmap)=}")
    rospy.loginfo(f"{len([cell for cell in costmap if cell <= 150])=}")
    start_cell = Cell(start, 0, euclidean_dist(0, goal, width), None)
    to_visit = [start_cell]  # array of cells not position
    visited = []


    while to_visit:
        current_index = find_lowest_f(to_visit)
        current = to_visit.pop(current_index)
        visited.append(current)
        grid_visualisation.set_color(current.pos, "pale yellow")

        if current.pos == goal:
            return get_path(current)# call function to get path by using the parent cells

        all_neighbors = find_neighbors(
            current.pos, width, height, costmap, 5
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
            penalty = calculate_angle(current.parent.pos, current.pos, neighbor[0], width) / 10
            new_g_score = current.g + neighbor[1] + penalty
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

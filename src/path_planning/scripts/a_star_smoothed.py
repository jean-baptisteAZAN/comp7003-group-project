from __future__ import annotations
import math
from algorithms.neighbors import find_neighbors
import rospy
from a_star import Cell, find_lowest_f, euclidean_dist, get_path

MAX_LINEAR_VELOCITY = 0.22
MAX_ANGULAR_VELOCITY = 2.84


def calculate_angle(current: Cell, neighbor_pos: int, width):
    previous = current.parent
    if previous is None:
        return 0

    previous_x, previous_y = previous.pos % width, previous.pos // width
    current_x, current_y = current.pos % width, current.pos // width
    neighbor_x, neighbor_y = neighbor_pos % width, neighbor_pos // width

    ux, uy = previous_x - current_x, previous_y - current_y
    vx, vy = neighbor_x - current_x, neighbor_y - current_y

    dot = ux * vx + uy * vy
    cross = ux * vy - uy * vx

    return abs(
        math.atan2(cross, dot)
    )  # for a signed angle, then we get the absolute value


def cost_function(distance, angle):
    time_trans = distance / MAX_LINEAR_VELOCITY
    time_rot = angle / MAX_ANGULAR_VELOCITY

    return time_trans + time_rot



def bresenham_cells(start, end, width):
    # This algorithm returns the list of cells between 2 cells

    start_x, start_y = start % width, start // width
    end_x, end_y = end % width, end // width

    traversed_cells = []

    delta_x = abs(end_x - start_x)
    delta_y = abs(end_y - start_y)

    # direction
    step_x = 1 if end_x >= start_x else -1
    step_y = 1 if end_y >= start_y else -1

    current_x, current_y = start_x, start_y

    # if the line is more horizontal
    if delta_x >= delta_y:
        error_accumulator = delta_x // 2

        while current_x != end_x:
            traversed_cells.append(current_x + current_y * width)
            error_accumulator -= delta_y
            if error_accumulator < 0:
                #if we are too far away in the y axis, we add a step in this direction
                current_y += step_y
                error_accumulator += delta_x

            current_x += step_x

    # if the line is more vertical
    else:
        error_accumulator = delta_y // 2

        while current_y != end_y:
            traversed_cells.append(current_x+ current_y * width)

            error_accumulator -= delta_x
            if error_accumulator < 0:
                current_x += step_x
                error_accumulator += delta_y

            current_y += step_y

    traversed_cells.append(end_x + end_y * width)
    return traversed_cells

def path_smoothing(path, costmap, width):
    smoothed_path = [path[0]]

    ref = path[0]
    for i in range(2, len(path) - 1):
        next_point = path[i]
        accessible = True
        for cell in bresenham_cells(ref, next_point, width):
            if costmap[cell] > 150:
                accessible = False
                break
        if accessible == False:
            smoothed_path.append(path[i - 1])
            ref = path[i - 1]

    
    smoothed_path.append(path[-1])

    return smoothed_path




def a_star_smoothed(
    start, goal, width, height, costmap, resolution, origin, grid_visualisation
):
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
            path = get_path(current) # call function to get path by using the parent cells
            rospy.loginfo(f"{len(path)=}")
            final_path = path_smoothing(path, costmap, width) # line of sight smoothing
            rospy.loginfo(f"{len(final_path)=}")
            return final_path

        all_neighbors = find_neighbors(
            current.pos, width, height, costmap, 1
        )  # 1 for the last param because otherwise, the penalty related to the costmap would be too impactful

        for neighbor in all_neighbors:
            # check if neighbor in visited, if yes, skip (continue)
            if neighbor[0] in visited:
                continue

            # check if neighbor already in to_visit, if yes, check if g_score is smaller, if yes, update cell
            # if not, create new Cell corresponding to neighbor and add it to to_visit
            angle = calculate_angle(current, neighbor[0], width)
            time_cost = cost_function(neighbor[1], angle)
            new_h_score = euclidean_dist(neighbor[0], goal, width) / MAX_LINEAR_VELOCITY

            new_g_score = (
                current.g + time_cost + angle
            )  # add travel time as cost + penalty
            new_f_score = new_g_score + new_h_score

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

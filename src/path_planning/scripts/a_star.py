from __future__ import annotations
import math
from algorithms.neighbors import find_neighbors
import rospy
from gridviz import GridViz

class Cell:
    def __init__(self, pos: int, g: int, f: int, parent: Cell):
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

    while last_cell.parent is not  None:
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
    start_cell = Cell(start, 0, euclidean_dist(0, goal, width), None)
    to_visit = [start_cell]  # array of cells not position
    visited = []

    # import numpy as np
    # import matplotlib.pyplot as plt

    # def show_maze_with_point(pixel_list, width, height, point_index):
    #     """
    #     Displays the maze and marks a specific index with a red dot.
    #     """
    #     # 1. Convert list to NumPy array
    #     pixel_array = np.array(pixel_list, dtype=np.float32)

    #     # 2. CLEANUP: Fix the 'noise' lines
    #     # If -1 (unknown) is becoming white, set it to 127 (gray) or 0 (black)
    #     pixel_array[pixel_array == -1] = 0 
    #     pixel_array[pixel_array == 255] = 0 # Often costmaps use 255 for unknown

    #     # 3. Reshape - MUST be (height, width) to avoid shearing lines
    #     image_matrix = pixel_array.reshape((height, width))

    #     # 4. Calculate X and Y from the 1D index
    #     x = point_index % width
    #     y = point_index // width

    #     # 5. Plotting
    #     plt.figure(figsize=(8, 8))
        
    #     # Use origin='lower' so (0,0) is the bottom-left, matching ROS convention
    #     plt.imshow(image_matrix, cmap='gray', origin='lower')
        
    #     # Overlay the red point
    #     plt.scatter(x, y, color='red', s=50, label=f'Point at {point_index}')
        
    #     plt.title(f"Maze Visualization ({width}x{height})")
    #     plt.legend()
    #     plt.axis('on') # Keep axis on to verify coordinates
    #     plt.show()
    # show_maze_with_point(costmap, width=width, height=height, point_index=start)

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

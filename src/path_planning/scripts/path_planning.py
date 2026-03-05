#!/usr/bin/env python3
from __future__ import annotations
from a_star import a_star
from a_star_smoothed import a_star_smoothed
import math
import threading
import rospy
import csv
import os
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gridviz import GridViz
import numpy as np
import matplotlib.pyplot as plt

# Performance metrics CSV setup
METRICS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "metrics")
CSV_HEADER = ["algorithm", "segment", "start", "goal", "path_length", "computation_time_s", "execution_time_s"]


def metrics_file_for(algo_name):
    """Return the CSV path for a given algorithm (e.g. metrics_standard.csv)."""
    return os.path.join(METRICS_DIR, f"metrics_{algo_name}.csv")

ARRIVAL_THRESHOLD = 0.3  # meters


def grid_to_world(grid_index, width, resolution, origin):
    """Convert a 1D grid index to world (x, y) coordinates."""
    gx = grid_index % width
    gy = grid_index // width
    wx = origin[0] + (gx + 0.5) * resolution
    wy = origin[1] + (gy + 0.5) * resolution
    return wx, wy


def get_robot_position():
    """Get current robot position from /odom (blocking, single message)."""
    msg = rospy.wait_for_message("/odom", Odometry, timeout=10.0)
    return msg.pose.pose.position.x, msg.pose.pose.position.y


def track_execution_times(algo_name, goals, width, resolution, origin):
    """Run in a separate thread. Waits for the robot to reach each goal and logs execution time."""
    goals_world = [grid_to_world(g, width, resolution, origin) for g in goals]
    rate = rospy.Rate(10)

    for segment_idx, (goal_x, goal_y) in enumerate(goals_world, start=1):
        t_start = rospy.Time.now()
        rospy.loginfo("[%s] Segment %d: Waiting for robot to reach goal...", algo_name, segment_idx)

        while not rospy.is_shutdown():
            robot_x, robot_y = get_robot_position()
            dist = math.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)
            if dist < ARRIVAL_THRESHOLD:
                exec_time = (rospy.Time.now() - t_start).to_sec()
                rospy.loginfo(
                    "[%s] Segment %d: Robot arrived | execution_time=%.2fs",
                    algo_name, segment_idx, exec_time
                )
                # Update the CSV row for this segment
                update_execution_time_in_csv(algo_name, segment_idx, exec_time)
                break
            rate.sleep()

    rospy.loginfo("[%s] All segments completed.", algo_name)


def update_execution_time_in_csv(algorithm, segment, exec_time_s):
    """Update the execution_time_s column for a given algorithm/segment row."""
    filepath = metrics_file_for(algorithm)
    rows = []
    with open(filepath, "r") as f:
        rows = list(csv.reader(f))

    for row in rows[1:]:
        if row[0] == algorithm and int(row[1]) == segment:
            row[6] = round(exec_time_s, 4)
            break

    with open(filepath, "w") as f:
        csv.writer(f).writerows(rows)


def init_metrics_csv(algo_name):
    """Create metrics directory and CSV file with header for a given algorithm."""
    if not os.path.exists(METRICS_DIR):
        os.makedirs(METRICS_DIR)
    filepath = metrics_file_for(algo_name)
    with open(filepath, "w") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
    rospy.loginfo("Metrics CSV initialised: %s", filepath)


def write_metric(algorithm, segment, start, goal, path_length, computation_time_s, execution_time_s=None):
    """Append one row of metrics to the CSV and log to ROS."""
    exec_str = round(execution_time_s, 4) if execution_time_s is not None else ""
    row = [algorithm, segment, start, goal, path_length, round(computation_time_s, 6), exec_str]
    filepath = metrics_file_for(algorithm)
    with open(filepath, "a") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    exec_log = "%.2fs" % execution_time_s if execution_time_s is not None else "N/A"
    rospy.loginfo(
        "[%s] Segment %d (%d -> %d) | path_length=%d | comp_time=%.4fs | exec_time=%s",
        algorithm, segment, start, goal, path_length, computation_time_s, exec_log
    )


def run_algorithm(algo_func, algo_name, start, goals, width, height, costmap, resolution, origin):
    """Run a pathfinding algorithm on all segments and collect metrics.
    Returns the final concatenated path."""
    final_path = []
    current_start = start

    for segment_idx, goal in enumerate(goals, start=1):
        if final_path:
            current_start = final_path[-1]

        grid_visualisation = GridViz(costmap, resolution, origin, current_start, goal, width)

        # Measure computation time
        t_start = rospy.Time.now()
        path = algo_func(
            current_start, goal, width, height, costmap, resolution, origin, grid_visualisation
        )
        t_end = rospy.Time.now()
        computation_time = (t_end - t_start).to_sec()

        if not path:
            rospy.logwarn("[%s] Segment %d: No path found (%d -> %d)", algo_name, segment_idx, current_start, goal)
            write_metric(algo_name, segment_idx, current_start, goal, 0, computation_time)
        else:
            write_metric(algo_name, segment_idx, current_start, goal, len(path), computation_time)
            final_path.extend(path)
            rospy.loginfo("[%s] Segment %d: Path sent to navigation stack", algo_name, segment_idx)
    return final_path


def make_plan(req) -> PathPlanningPluginResponse:
    """
    Callback function used by the service server to process
    requests from clients. It returns a msg of type PathPlanningPluginResponse
    """
    costmap: list[int] = req.costmap_ros
    # number of columns in the occupancy grid
    width: int = req.width
    # number of rows in the occupancy grid
    height: int = req.height
    start: int = req.start
    goal: int = req.goal
    # side of each grid map square in meters
    resolution = 0.05
    origin: list[int] = [-3.312564, -3.270421, 0.000000]

    rospy.loginfo(f"//// {goal=}")

    def show_maze_with_point(pixel_list, width, height, point_index):
        """
        Displays the maze and marks a specific index with a red dot.
        """
        # 1. Convert list to NumPy array
        pixel_array = np.array(pixel_list, dtype=np.float32)

        # 2. CLEANUP: Fix the 'noise' lines
        # If -1 (unknown) is becoming white, set it to 127 (gray) or 0 (black)
        pixel_array[pixel_array == -1] = 0 
        pixel_array[pixel_array == 255] = 0 # Often costmaps use 255 for unknown

        # 3. Reshape - MUST be (height, width) to avoid shearing lines
        image_matrix = pixel_array.reshape((height, width))

        # 4. Calculate X and Y from the 1D index
        x = point_index % width
        y = point_index // width

        # 5. Plotting
        plt.figure(figsize=(8, 8))
        
        # Use origin='lower' so (0,0) is the bottom-left, matching ROS convention
        plt.imshow(image_matrix, cmap='gray', origin='lower')
        
        # Overlay the red point
        plt.scatter(x, y, color='red', s=50, label=f'Point at {point_index}')
        
        plt.title(f"Maze Visualization ({width}x{height})")
        plt.legend()
        plt.axis('on') # Keep axis on to verify coordinates
        plt.show()
    # This function was used to debug our costmap when the AMCL was not aligned
    # show_maze_with_point(costmap, width=width, height=height, point_index=start)


    goals = [
        5717,
        8436,
        13079,
        17625
    ]
    # This goals are indexes, we got those values by logging req.goal and clicking with 2D Nav Goal in Rviz
    algo_param = rospy.get_param("~algorithm", "standard")
    available_algorithms = {
        "standard": ("standard", a_star),
        "smoothed": ("smoothed", a_star_smoothed),
    }

    if algo_param in available_algorithms:
        algo_name, algo_func = available_algorithms[algo_param]
    else:
        rospy.logwarn(f"Unknown algorithm '{algo_param}', defaulting to standard", )
        algo_name, algo_func = available_algorithms["standard"]

    rospy.loginfo(f"Selected algorithm: {algo_name}")

    init_metrics_csv(algo_name)
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Running algorithm: {algo_name}")
    rospy.loginfo("=" * 60)
    path = run_algorithm(algo_func, algo_name, start, goals, width, height, costmap, resolution, origin)

    # Start execution time tracking in a background thread
    tracker_thread = threading.Thread(
        target=track_execution_times,
        args=(algo_name, goals, width, resolution, origin),
        daemon=True,
    )
    tracker_thread.start()
    rospy.loginfo(f"Execution time tracking started for {len(goals)} goals")

    resp = PathPlanningPluginResponse()
    resp.plan = path
    return resp


def clean_shutdown():
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("path_planning_server", log_level=rospy.INFO, anonymous=False)
    rospy.loginfo("Start")
    make_plan_service = rospy.Service(
        "/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan
    )
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    rospy.on_shutdown(clean_shutdown)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2), rospy.signal_shutdown("Shutting down"), oneshot=True)

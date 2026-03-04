#!/usr/bin/env python3
from __future__ import annotations
from a_star import a_star
from a_star_enhanced import a_star_smoothed
import math
import rospy
import csv
import os
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gridviz import GridViz

# --- Performance metrics CSV setup ---
METRICS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "metrics")
METRICS_FILE = os.path.join(METRICS_DIR, "metrics.csv")
CSV_HEADER = ["algorithm", "segment", "start", "goal", "path_length", "computation_time_s", "execution_time_s"]

# --- Execution time tracking ---
# Stores goal waypoints as world coords for the current run, populated in make_plan
execution_tracker = {
    "goals_world": [],       # [(x, y), ...] goal positions in world frame
    "algorithm": "",
    "nav_start_time": None,
    "current_goal_idx": 0,
    "arrival_threshold": 0.3,  # meters - robot considered arrived when within this distance
    "resolution": 0.05,
    "origin": [0, 0, 0],
    "width": 0,
}


def grid_to_world(grid_index, width, resolution, origin):
    """Convert a 1D grid index to world (x, y) coordinates."""
    gx = grid_index % width
    gy = grid_index // width
    wx = origin[0] + (gx + 0.5) * resolution
    wy = origin[1] + (gy + 0.5) * resolution
    return wx, wy


def amcl_pose_callback(msg):
    """Monitor robot position to detect arrival at each goal waypoint."""
    tracker = execution_tracker
    if tracker["nav_start_time"] is None:
        return
    if tracker["current_goal_idx"] >= len(tracker["goals_world"]):
        return

    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    goal_x, goal_y = tracker["goals_world"][tracker["current_goal_idx"]]

    dist = math.sqrt((robot_x - goal_x) ** 2 + (robot_y - goal_y) ** 2)

    if dist < tracker["arrival_threshold"]:
        arrival_time = rospy.Time.now()
        exec_time = (arrival_time - tracker["nav_start_time"]).to_sec()
        segment = tracker["current_goal_idx"] + 1

        # Append execution time to the CSV
        append_execution_time(tracker["algorithm"], segment, exec_time)

        rospy.loginfo(
            "[%s] Segment %d: Robot arrived | execution_time=%.2fs",
            tracker["algorithm"], segment, exec_time
        )

        # Next segment starts now
        tracker["nav_start_time"] = arrival_time
        tracker["current_goal_idx"] += 1


def init_metrics_csv():
    """Create metrics directory and CSV file with header if needed."""
    if not os.path.exists(METRICS_DIR):
        os.makedirs(METRICS_DIR)
    with open(METRICS_FILE, "w") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
    rospy.loginfo("Metrics CSV initialised: %s", METRICS_FILE)


def write_metric(algorithm, segment, start, goal, path_length, computation_time_s):
    """Append one row of planning metrics to the CSV (execution_time filled later)."""
    row = [algorithm, segment, start, goal, path_length, round(computation_time_s, 6), ""]
    with open(METRICS_FILE, "a") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    rospy.loginfo(
        "[%s] Segment %d (%d -> %d) | path_length=%d | comp_time=%.4fs",
        algorithm, segment, start, goal, path_length, computation_time_s
    )


def append_execution_time(algorithm, segment, exec_time_s):
    """Update the CSV row for the given algorithm/segment with execution time."""
    rows = []
    with open(METRICS_FILE, "r") as f:
        reader = csv.reader(f)
        rows = list(reader)

    for row in rows[1:]:  # skip header
        if row[0] == algorithm and int(row[1]) == segment:
            row[6] = round(exec_time_s, 4)
            break

    with open(METRICS_FILE, "w") as f:
        writer = csv.writer(f)
        writer.writerows(rows)


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

    import numpy as np
    import matplotlib.pyplot as plt

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
    # show_maze_with_point(costmap, width=width, height=height, point_index=start)


    goals = [
        goal
        # 17506,
        # 2222,
        # 1791,
        # 16999,
    ]
    init_metrics_csv()

    algo_param = rospy.get_param("~algorithm", "standard")
    available_algorithms = {
        "standard": ("standard", a_star),
        "smoothed": ("smoothed", a_star_smoothed),
    }

    if algo_param == "both":
        algorithms = [available_algorithms["standard"], available_algorithms["smoothed"]]
    elif algo_param in available_algorithms:
        algorithms = [available_algorithms[algo_param]]
    else:
        rospy.logwarn("Unknown algorithm '%s', defaulting to standard", algo_param)
        algorithms = [available_algorithms["standard"]]

    rospy.loginfo("Selected algorithm(s): %s", [a[0] for a in algorithms])
    final_path = []
    for algo_name, algo_func in algorithms:
        rospy.loginfo("=" * 60)
        rospy.loginfo("Running algorithm: %s", algo_name)
        rospy.loginfo("=" * 60)
        path = run_algorithm(algo_func, algo_name, start, goals, width, height, costmap, resolution, origin)
        # Use the last algorithm's path as the one sent to move_base
        final_path = path

    last_algo_name = algorithms[-1][0]
    goals_world = [grid_to_world(g, width, resolution, origin) for g in goals]
    execution_tracker["goals_world"] = goals_world
    execution_tracker["algorithm"] = last_algo_name
    execution_tracker["nav_start_time"] = rospy.Time.now()
    execution_tracker["current_goal_idx"] = 0
    execution_tracker["resolution"] = resolution
    execution_tracker["origin"] = origin
    execution_tracker["width"] = width
    rospy.loginfo("Execution time tracking started for %d goals", len(goals))

    resp = PathPlanningPluginResponse()
    resp.plan = final_path
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
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.on_shutdown(clean_shutdown)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2), rospy.signal_shutdown("Shutting down"), oneshot=True)

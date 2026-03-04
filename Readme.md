## launching:

In first terminal:
`source devel/setup.bash`
`roslaunch custom_gazebo_world custom_world.launch`

In second terminal:
`source devel/setup.bash`
`roslaunch path_planning turtlebot3_custom_world.launch algorithm:=smoothed`
algorithm can be either 'smoothed' or 'standard'
In rviz: Click '2D Nav Goal' and click somewhere on the map to invoke the python node.
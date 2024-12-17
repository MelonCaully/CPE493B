source install/setup.bash
colcon build

(Each number in a seperate terminal)
1. ros2 launch f1tenth_gym_ros gym_bridge_launch.py

Any order for these:
    Add MarkerArray on rviz
2. ros2 run pure_pursuit_pkg pure_pursuit
    Turn off 'markers' in MarkerArray on rviz

3. ros2 run gap_pkg reactive_node
4. ros2 run safety_pkg safety_node

Last:
5. ros2 run filter_node filter_node

Troubleshotting:
- Sometimes, if you get an error that says "TypeError" the fix is to run it again lol.
- In 'CPE493B/f1tenth_gym_ros/config/sim.yaml', make sure the `map_path` varaible is going to the correct directory
    ie. `/Path/To/Director/project/src/CPE493B/f1tenth_gym_ros/maps/[desired_map]`

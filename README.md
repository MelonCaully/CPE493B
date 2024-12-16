First
1. ros2 launch f1tenth_gym_ros gym_bridge_launch.py

Any order for these:
2. ros2 run pure_pursuit_pkg pure_pursuit
3. ros2 run gap_pkg reactive_node
4. ros2 run safety_pkg safety_node

Last:
5. ros2 run filter_node filter_node
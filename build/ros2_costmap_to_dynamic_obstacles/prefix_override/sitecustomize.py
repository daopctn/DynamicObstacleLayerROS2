import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/daopctn/ros2_ws/install/ros2_costmap_to_dynamic_obstacles'

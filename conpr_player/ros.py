"""Public compatibility imports for the selected, already sourced ROS environment."""
from .messages import *
from .backend import spin_executor
from .environment import ROS1
if ROS1:
    from .ros1 import PlayerNode
else:
    from .ros2 import PlayerNode

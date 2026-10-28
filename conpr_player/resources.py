from pathlib import Path
from .environment import ROS1

def config_path(name):
    if ROS1:
        import rospkg
        share=Path(rospkg.RosPack().get_path('conpr_file_player')).resolve()
    else:
        from ament_index_python.packages import get_package_share_directory
        share=Path(get_package_share_directory('conpr_file_player'))
    path=share/'config'/name
    if not path.exists():
        # Catkin devel space points rospack at the wrapper source directory.
        path=share.parent.parent/'config'/name
    if not path.exists():raise FileNotFoundError(path)
    return path

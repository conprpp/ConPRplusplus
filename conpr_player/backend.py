"""Small lifecycle adapter; each process imports only its active ROS implementation."""
import sys
import threading
from .environment import ROS1
if not ROS1:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException,ShutdownException
    def init(): rclpy.init(args=None)
    def ok(): return rclpy.ok()
    def shutdown():
        if rclpy.ok(): rclpy.shutdown()
    def cli_args():
        from rclpy.utilities import remove_ros_args
        return remove_ros_args(sys.argv)[1:]
    def spin_executor(executor):
        try: executor.spin()
        except (ExternalShutdownException,ShutdownException): pass
else:
    import rospy
    from dataclasses import dataclass
    @dataclass
    class Parameter:
        name: str
        value: object
    def init():
        try: rospy.get_master().getPid()
        except Exception as exc:
            raise RuntimeError('ROS master unavailable. Use roslaunch, or start roscore before direct execution.') from exc
        rospy.init_node('conpr_file_player',disable_signals=True)
    def ok(): return not rospy.is_shutdown()
    def shutdown(): rospy.signal_shutdown('Player closed')
    def cli_args(): return rospy.myargv(argv=sys.argv)[1:]
    class SingleThreadedExecutor:
        # rospy dispatches subscriber/service callbacks itself. This wait thread only
        # provides the common GUI lifecycle and must NOT stop rospy on sequence reload.
        def __init__(self): self.done=threading.Event()
        def add_node(self,node): pass
        def spin(self):
            while not self.done.wait(.1) and ok(): pass
        def shutdown(self,timeout_sec=None): self.done.set()
    def spin_executor(executor): executor.spin()

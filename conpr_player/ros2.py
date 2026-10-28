"""Native ROS 2 transport for Humble and Jazzy."""
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_srvs.srv import Trigger,SetBool
from rcl_interfaces.msg import SetParametersResult
from .messages import *
from .node_control import NodeControl

class PlayerNode(NodeControl,Node):
    parameter_result=SetParametersResult
    def __init__(self,factory):
        super().__init__('conpr_file_player',namespace=factory.options.namespace)
        self.factory=factory;self.controller=None;self.message_counts={}
        reliable=ReliabilityPolicy.RELIABLE if factory.options.reliable else ReliabilityPolicy.BEST_EFFORT
        self.qos=QoSProfile(depth=256,reliability=reliable)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        classes={'clock':Clock,'points':PointCloud2,'camera':Image,'info':CameraInfo,'imu':Imu,'pose':PoseStamped,'path':Path,'tf':TFMessage,'static':TFMessage}
        if factory.options.custom:classes['custom']=factory.CustomMsg
        self.pubs={k:self.create_publisher(v,factory.topic(k),latched if k in ('static','path') else self.qos) for k,v in classes.items()}
        self.status_pub=self.create_publisher(String,'conpr/player/status',10)
        self.timer=self.create_timer(.5,self._status)
        for name in ('play','stop','restart','step'):
            self.create_service(Trigger,'conpr/player/'+name,self._trigger(name))
        self.create_service(SetBool,'conpr/player/pause',self._pause)
        for n,v in [('rate',1.),('seek_s',0.),('loop',False),('camera',factory.options.camera),('lidar',factory.options.lidar),('imu',factory.options.imu),('gt',factory.options.gt),('gt_tf',factory.options.gt_tf)]:self.declare_parameter(n,v)
        self.add_on_set_parameters_callback(self._apply_parameters)

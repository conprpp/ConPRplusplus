"""Native ROS 1 Noetic transport; no ROS bridge or ROS 2 runtime is involved."""
import threading
from dataclasses import dataclass
import rospy
from std_srvs.srv import Trigger,TriggerResponse,SetBool,SetBoolResponse
from .messages import *
from .node_control import NodeControl

@dataclass
class ParameterResult:
    successful: bool
    reason: str=''

class PlayerNode(NodeControl):
    parameter_result=ParameterResult
    def __init__(self,factory):
        self.factory=factory;self.controller=None;self.message_counts={};self.qos=None
        self.services=[];self.closed=threading.Event()
        classes={'clock':Clock,'points':PointCloud2,'camera':Image,'info':CameraInfo,'imu':Imu,'pose':PoseStamped,'path':Path,'tf':TFMessage,'static':TFMessage}
        if factory.options.custom: classes['custom']=factory.CustomMsg
        self.pubs={k:rospy.Publisher(factory.topic(k),v,queue_size=256,latch=k in ('static','path')) for k,v in classes.items()}
        self.status_pub=rospy.Publisher(self.control_name('status'),String,queue_size=10)
        for name in ('play','stop','restart','step'):
            self.services.append(rospy.Service(self.control_name(name),Trigger,
                lambda req,n=name:self._trigger(n)(req,TriggerResponse())))
        self.services.append(rospy.Service(self.control_name('pause'),SetBool,
            lambda req:self._pause(req,SetBoolResponse())))
        self.param_values=dict(rate=1.,seek_s=0.,loop=False,camera=factory.options.camera,lidar=factory.options.lidar,
                              imu=factory.options.imu,gt=factory.options.gt,gt_tf=factory.options.gt_tf)
        for k,v in self.param_values.items():rospy.set_param(self.param_name(k),v)
        self.services.append(rospy.Service(self.control_name('apply_parameters'),Trigger,self.apply_server_parameters))
        self.status_thread=threading.Thread(target=self.status_loop,daemon=True);self.status_thread.start()
    def control_name(self,name):
        ns=self.factory.options.namespace.strip('/')
        return '/'+(ns+'/' if ns else '')+'conpr/player/'+name
    def param_name(self,name):return self.control_name('parameters/'+name)
    def status_loop(self):
        # Wall time: a paused /clock must not freeze status/control.
        while not self.closed.wait(.5) and not rospy.is_shutdown():self._status()
    def set_parameters(self,params):
        result=self._apply_parameters(params)
        if result.successful:
            for p in params:self.param_values[p.name]=p.value;rospy.set_param(self.param_name(p.name),p.value)
        return [result]
    def apply_server_parameters(self,req):
        from .backend import Parameter
        params=[Parameter(k,rospy.get_param(self.param_name(k),v)) for k,v in self.param_values.items()]
        changed=[p for p in params if p.value!=self.param_values[p.name]]
        result=self.set_parameters(changed)[0]
        return TriggerResponse(success=result.successful,message=result.reason or 'Applied')
    def create_publisher(self,cls,topic,qos):return rospy.Publisher(topic,cls,queue_size=256)
    def destroy_node(self):
        self.closed.set();self.status_thread.join(timeout=2)
        for srv in self.services:srv.shutdown('Sequence unloaded')
        for pub in self.pubs.values():pub.unregister()
        self.status_pub.unregister()

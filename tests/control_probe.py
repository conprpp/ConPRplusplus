"""Exercise native services/parameters and invalid-control rejection on all backends."""
import argparse,json,threading,time
from pathlib import Path
from conpr_player import backend
from conpr_player.environment import ROS1,ROS_DISTRO
from conpr_player.dataset import Dataset
from conpr_player.controller import Playback
from conpr_player.ros import Options,MessageFactory,PlayerNode
from std_srvs.srv import Trigger,SetBool

p=argparse.ArgumentParser();p.add_argument('--dataset',required=True);p.add_argument('--output',required=True);a=p.parse_args()
backend.init();ds=Dataset(a.dataset);node=PlayerNode(MessageFactory(ds,Options()))
ctl=Playback(ds,node.emit,node.reset,end_s=min(5.,ds.duration));node.controller=ctl
ex=backend.SingleThreadedExecutor();ex.add_node(node)
if not ROS1:
    from rclpy.node import Node
    from rcl_interfaces.srv import SetParameters
    from rclpy.parameter import Parameter
    client=Node('conpr_controls_test');ex.add_node(client)
else:import rospy
thread=threading.Thread(target=backend.spin_executor,args=(ex,),daemon=True);thread.start()

def wait(fn):
    deadline=time.monotonic()+10
    while not fn():
        assert time.monotonic()<deadline,'Timeout'
        time.sleep(.01)

def call(kind,name,**kwargs):
    if ROS1:
        rospy.wait_for_service(name,timeout=5)
        return rospy.ServiceProxy(name,kind)(**kwargs)
    c=client.create_client(kind,name);assert c.wait_for_service(timeout_sec=5)
    future=c.call_async(kind.Request(**kwargs));wait(future.done);res=future.result();client.destroy_client(c);return res

def parameters(values):
    if ROS1:
        for k,v in values.items():rospy.set_param('/conpr/player/parameters/'+k,v)
        return call(Trigger,'/conpr/player/apply_parameters').success
    res=call(SetParameters,'/conpr_file_player/set_parameters',parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()])
    return all(x.successful for x in res.results)
try:
    assert call(Trigger,'/conpr/player/play').success;wait(lambda:ctl.counts['cam']>=2)
    assert call(SetBool,'/conpr/player/pause',data=True).success
    n=dict(ctl.counts);time.sleep(.15);assert n==dict(ctl.counts)
    seek=min(1.,ds.duration/2)
    assert parameters(dict(rate=2.,seek_s=seek,gt=True,loop=True))
    assert ctl.rate==2 and ctl.loop and ctl.enabled['gt']
    assert ctl.position==next(e.stamp for e in ds.events if e.stamp>=ds.start+round(seek*1e9))
    assert call(Trigger,'/conpr/player/step').success;wait(lambda:ctl.state=='PAUSED')
    assert not parameters(dict(rate=0.));assert ctl.rate==2
    assert call(Trigger,'/conpr/player/stop').success;assert ctl.state=='STOPPED'
    Path(a.output).write_text(json.dumps(dict(passed=True,ros_distro=ROS_DISTRO,play_pause=True,seek_rate_gt_loop=True,step_stop=True,invalid_rate_rejected=True),indent=2)+'\n')
    print('CONTROL PASS',ROS_DISTRO)
finally:
    ctl.close();ex.shutdown(timeout_sec=3);thread.join(3);node.destroy_node()
    if not ROS1:client.destroy_node()
    backend.shutdown()

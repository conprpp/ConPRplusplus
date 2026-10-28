"""Offline GT lookahead must preserve original poses, sensor stamps and export construction."""
from types import SimpleNamespace
import numpy as np
from conpr_player.node_control import NodeControl
from conpr_player.messages import MessageFactory,Options
from conpr_player.dataset import Event
from conpr_player.environment import ROS1

def ns(s):
    return s.to_nsec() if ROS1 else s.sec*1_000_000_000+s.nanosec

class Recorder(NodeControl):
    def __init__(self):
        self.sent=[]
        times=np.arange(1_000_000_000,3_000_000_001,100_000_000,dtype=np.int64)
        values=np.zeros((len(times),7));values[:,0]=np.arange(len(times));values[:,6]=1
        ds=SimpleNamespace(gt_ts=times,gt_values=values,
            camera=dict(image_height=480,image_width=640,distortion_model='plumb_bob',
                distortion_coefficients={'data':[0]*5},camera_matrix={'data':list(np.eye(3).flat)},
                rectification_matrix={'data':list(np.eye(3).flat)},projection_matrix={'data':list(np.zeros((3,4)).flat)}),
            extrinsics={'T_imu_lidar':np.eye(4),'T_camera_lidar':np.eye(4)})
        self.factory=MessageFactory(ds,Options(gt=True,gt_tf=True))
    def publish_messages(self,items):self.sent.extend(items)

def test_gt_tf_lookahead_keeps_original_poses_and_is_bounded():
    n=Recorder();t=1_050_000_000;n.reset(t)
    transforms=[x for k,m in n.sent if k=='tf' for x in m.transforms]
    assert [ns(x.header.stamp) for x in transforms]==list(range(1_000_000_000,1_500_000_001,100_000_000))
    for j,x in enumerate(transforms):
        assert x.transform.translation.x==j and x.transform.rotation.w==1
    n.sent=[]
    e=Event(t,'imu');data=np.array([0,0,0,1,1,2,3,4,5,6.])
    expected=MessageFactory(n.factory.ds,Options(gt=True,gt_tf=True)).messages(e,data)
    n.emit(e,data)
    assert n.sent==expected  # same sensor values, stamps and clock; no repeated TF

def test_rewind_republishes_and_disable_stops_prefetch():
    n=Recorder();n.reset(2_000_000_000);n.sent=[];n.reset(1_050_000_000)
    assert min(ns(x.header.stamp) for k,m in n.sent if k=='tf' for x in m.transforms)==1_000_000_000
    n.factory.options.gt_tf=False;n.sent=[];assert not n._preview_gt_tf(1_100_000_000)
    assert not n.sent
    n.factory.options.gt_tf=True;assert n._preview_gt_tf(1_100_000_000)
    assert n.sent

def test_export_factory_keeps_event_timed_gt_tf():
    n=Recorder();e=Event(1_000_000_000,'gt')
    msgs=dict(n.factory.messages(e,n.factory.ds.gt_values[0]))
    assert len(msgs['tf'].transforms)==1
    assert ns(msgs['tf'].transforms[0].header.stamp)==e.stamp

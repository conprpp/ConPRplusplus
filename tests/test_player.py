import csv,json,time,threading,os
from pathlib import Path
import numpy as np
import pytest
from conpr_player.dataset import Dataset,Event,POINT_DTYPE,load_intervals,discover
from conpr_player.controller import Playback

from conpr_player.environment import ROS1
@pytest.fixture(scope='module')
def ds(data_root):return Dataset(data_root/'Hall/260905')

def wait_for(fn,timeout=4):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if fn():return
        time.sleep(.005)
    raise AssertionError('Timeout waiting for condition')

def test_release_format(ds):
    from collections import Counter
    with (ds.path/'stamps.csv').open(newline='') as f:
        rows=list(csv.DictReader(f))
    assert ds.counts==dict(Counter(row['sensor'] for row in rows))
    assert POINT_DTYPE.itemsize==19
    assert ds.bounds()[1]==len(ds.events)
    with (ds.path/'GT_pose/poses.csv').open(newline='') as f:
        poses=list(csv.DictReader(f))
    assert ds.gt_values.shape==(len(poses),7)

def test_discovery_accepts_partial_download_and_explicit_folders(data_root,tmp_path):
    root=tmp_path/'download';site=root/'Hall';site.mkdir(parents=True)
    (site/'Calibration').symlink_to(data_root/'Hall/Calibration',target_is_directory=True)
    session=site/'260905';session.mkdir()
    for child in (data_root/'Hall/260905').iterdir():
        (session/child.name).symlink_to(child,target_is_directory=child.is_dir())
    assert discover(root)==[site/'260905']
    assert discover(site)==[site/'260905']
    assert discover(session)==[session]
    assert Dataset(session).counts==Dataset(data_root/'Hall/260905').counts
    assert discover('')==[]

class Fake:
    start=1_000_000_000;end=2_000_000_000;duration=1.
    def __init__(self,slow=False):
        self.events=[Event(self.start+i*10_000_000,'livox' if i%10==0 else 'imu') for i in range(101)]
        self.times=np.array([e.stamp for e in self.events]);self.lidar_indices=np.array([i for i,e in enumerate(self.events) if e.sensor=='livox']);self.slow=slow
    def bounds(self,a=0,b=None):return 0,len(self.events)
    def decode(self,e):
        if self.slow:time.sleep(.003)
        return e.stamp

def test_pause_rate_seek_step_loop():
    emitted=[];resets=[];ds=Fake();ctl=Playback(ds,lambda e,d:emitted.append(e),resets.append,rate=2)
    try:
        ctl.play();wait_for(lambda:len(emitted)>=10);ctl.pause();n=len(emitted);time.sleep(.03);assert len(emitted)==n
        ctl.set_rate(10);ctl.seek(.5);assert resets[-1]==1_500_000_000
        ctl.step();wait_for(lambda:ctl.state=='PAUSED');assert emitted[-1].stamp==1_500_000_000
        ctl.step();wait_for(lambda:ctl.state=='PAUSED');assert emitted[-1].stamp==1_600_000_000
        ctl.play();wait_for(lambda:ctl.state=='FINISHED');assert emitted[-1].stamp==ds.end
        ctl.stop();assert ctl.state=='STOPPED' and ctl.i==0
        ctl.loop=True;ctl.restart();wait_for(lambda:ctl.loops>=1);ctl.pause();assert len(resets)>=3
        with pytest.raises(ValueError):ctl.set_rate(0)
        with pytest.raises(ValueError):ctl.seek(-1)
    finally:ctl.close()

def test_seek_does_not_publish_stale_prefetch():
    emitted=[];ds=Fake(True);ctl=Playback(ds,lambda e,d:emitted.append(e.stamp),rate=1)
    try:
        ctl.play();wait_for(lambda:len(emitted)>2);ctl.pause();ctl.seek(.8);n=len(emitted);ctl.play();wait_for(lambda:len(emitted)>n)
        assert all(t>=1_800_000_000 for t in emitted[n:])
    finally:ctl.close()

def test_end_callback_once_per_pass():
    ends=[];ctl=Playback(Fake(),lambda e,d:None,rate=100,on_finish=ends.append)
    try:
        ctl.play();wait_for(lambda:ctl.state=='FINISHED');time.sleep(.02)
        assert ends==[Fake.end]
        ctl.restart();wait_for(lambda:ctl.state=='FINISHED')
        assert ends==[Fake.end,Fake.end]
    finally:ctl.close()

def test_marked_intervals():
    emitted=[];ds=Fake();ctl=Playback(ds,lambda e,d:emitted.append(e.stamp),rate=100)
    try:
        ctl.skip_intervals=[(.2,.8)];ctl.skip_marked=True;ctl.play();wait_for(lambda:ctl.state=='FINISHED')
        assert not any(1_200_000_000<=t<1_800_000_000 for t in emitted)
        assert ctl.skipped['marked_interval']==60
    finally:ctl.close()

def test_bad_data_strict_and_skip():
    class Bad(Fake):
        def decode(self,e):
            if e.stamp==self.start+10_000_000:raise ValueError('broken')
            return 1
    for skip in (False,True):
        ctl=Playback(Bad(),lambda e,d:None,rate=100,skip_invalid=skip)
        try:
            ctl.play();wait_for(lambda:ctl.state in ('ERROR','FINISHED'))
            assert ctl.state==('FINISHED' if skip else 'ERROR')
            assert len(ctl.errors)==1
        finally:ctl.close()

@pytest.mark.skipif(ROS1,reason='ROS1 message payloads covered by multiros_probe.py')
def test_message_payloads_and_tf(ds):
    from conpr_player.ros import MessageFactory,Options,quaternion
    from rclpy.serialization import serialize_message,deserialize_message
    f=MessageFactory(ds,Options(gt=True,gt_tf=True,custom=True))
    for sensor in ('cam','livox','imu','gt'):
        e=next(e for e in ds.events if e.sensor==sensor);data=ds.decode(e);msgs=dict(f.messages(e,data))
        for msg in msgs.values():assert deserialize_message(serialize_message(msg),type(msg))==msg
        if sensor=='cam':
            m=msgs['camera'];assert bytes(m.data)==data.tobytes();assert m.encoding=='rgb8';assert list(msgs['info'].k)==ds.camera['camera_matrix']['data']
        elif sensor=='livox':
            m=msgs['points'];dtype=np.dtype({'names':[x.name for x in m.fields],'formats':['<f4']*4+['u1']*3+['<u4'],'offsets':[x.offset for x in m.fields],'itemsize':m.point_step});pts=np.frombuffer(m.data,dtype=dtype)
            for n in data.dtype.names:assert np.array_equal(pts[n],data[n])
            custom=msgs['custom'];assert custom.timebase==e.stamp and custom.point_num==len(data)
            for k in (0,len(data)//2,len(data)-1):
                for name in data.dtype.names:assert getattr(custom.points[k],name)==data[name][k]
        elif sensor=='imu':
            m=msgs['imu'];assert [m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]==list(data[:4]);assert m.angular_velocity.x==data[4];assert m.linear_acceleration.z==data[9]
        elif sensor=='gt':
            assert msgs['pose'].header.frame_id=='conpr_gt_world';assert msgs['tf'].transforms[0].child_frame_id=='conpr_imu'
    assert np.allclose(quaternion(np.eye(3)),[0,0,0,1])
    # Independent quaternion->matrix round trip for both transform directions.
    for tf,T in zip(f.static(ds.start).transforms,[ds.extrinsics['T_imu_lidar'],ds.extrinsics['T_imu_lidar']@np.linalg.inv(ds.extrinsics['T_camera_lidar'])]):
        q=tf.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        assert np.allclose(R,T[:3,:3],atol=1e-5)
        assert np.allclose([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],T[:3,3])

@pytest.mark.skipif(ROS1,reason='ROS1 native bag roundtrip covered by multiros_probe.py')
def test_mcap_roundtrip(ds,tmp_path):
    from conpr_player.bag import export
    from conpr_player.ros import Options,MessageFactory
    import rosbag2_py
    from rclpy.serialization import serialize_message
    dest=tmp_path/'bag';options=Options(gt=True,custom=True)
    result=export(ds,dest,options,0,.25);assert result['complete']
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(dest),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    actual=[]
    while reader.has_next():actual.append(reader.read_next())
    factory=MessageFactory(ds,options);lo,hi=ds.bounds(0,.25)
    expected=[]
    for e in ds.events[lo:hi]:
        for key,msg in factory.messages(e,ds.decode(e)):
            if key not in ('clock','path'):expected.append((factory.topic(key),msg,e.stamp))
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    type_map={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()}
    filtered=[(t,deserialize_message(d,type_map[t]),ts) for t,d,ts in actual if t not in ('/clock','/tf_static','/conpr/gt/path')]
    assert filtered==expected
    with pytest.raises(FileExistsError):export(ds,dest)

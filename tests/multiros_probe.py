"""Integration test against a sourced, BUILT player (Noetic/Humble/Jazzy).

Needs a public ConPRplusplus root. Noetic: start an isolated roscore first.
Tests original payloads, live transport, native bag roundtrip and actual bag play.
"""
import argparse,collections,hashlib,io,json,os,subprocess,threading,time
from pathlib import Path
from conpr_player import backend
from conpr_player.environment import ROS1,ROS_DISTRO
from conpr_player.dataset import Dataset
from conpr_player.messages import MessageFactory,Options,TYPES,TransformStamped,header
from conpr_player.ros import PlayerNode
from conpr_player.controller import Playback
from conpr_player.bag import export


def serialize(msg):
    if ROS1:
        buff=io.BytesIO();msg.serialize(buff);return buff.getvalue()
    from rclpy.serialization import serialize_message
    return serialize_message(msg)


def canonical(value):
    # Compare message contents, not CDR padding bytes or ROS1 transport-generated Header.seq.
    if hasattr(value,'to_nsec'):return {'nanoseconds':int(value.to_nsec())}
    if type(value).__module__.startswith('numpy') and hasattr(value,'item') and getattr(value,'ndim',1)==0:return value.item()
    if hasattr(value,'__slots__'):
        fields={}
        for slot in value.__slots__:
            key=slot.lstrip('_')
            if key=='seq':continue
            data=getattr(value,slot)
            if key=='data' and isinstance(data,(bytes,bytearray,memoryview)) or (key=='data' and hasattr(value,'encoding')) or (key=='data' and hasattr(value,'point_step')):
                fields[key]=hashlib.sha256(bytes(data)).hexdigest()
            else:fields[key]=canonical(data)
        return fields
    if isinstance(value,(str,int,float,bool)) or value is None:return value
    return [canonical(v) for v in value]

def fingerprint(topic,msg,t):
    payload=json.dumps(canonical(msg),sort_keys=True,separators=(',',':')).encode()
    return topic,int(t),hashlib.sha256(payload).hexdigest()
def ns(t):return t.to_nsec() if ROS1 else t.sec*1_000_000_000+t.nanosec

def fingerprints(topic,msg,t):
    # Live GT TF can batch several original transforms in one message. Compare
    # each stamped transform, without depending on the transport batch layout.
    if hasattr(msg,'transforms'):
        return [fingerprint(topic,tf,ns(tf.header.stamp)) for tf in msg.transforms]
    return [fingerprint(topic,msg,t)]

def expected_live_records(ds,rows,end):
    expected=collections.Counter(record for topic,msg,t in rows if topic!='/tf'
                                 for record in fingerprints(topic,msg,t))
    lo,hi=ds.bounds(0,end)
    start=ds.events[lo].stamp
    limit=ds.events[hi-1].stamp+500_000_000
    stamps=[int(t) for t in ds.gt_ts if int(t)<=limit]
    predecessors=[t for t in stamps if t<=start]
    first=predecessors[-1] if predecessors else start
    for t in stamps:
        if t<first:continue
        pose=ds.gt[t]
        tf=TransformStamped(header=header(t,'conpr_gt_world'),child_frame_id='conpr_imu')
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,pose[:3])
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,pose[3:7])
        expected[fingerprint('/tf',tf,t)]+=1
    return expected

def expected_messages(ds,options,end):
    f=MessageFactory(ds,options);lo,hi=ds.bounds(0,end);rows=[]
    for k,m in f.reset(ds.events[lo].stamp):rows.append((f.topic(k),m,ds.events[lo].stamp))
    for e in ds.events[lo:hi]:
        for k,m in f.messages(e,ds.decode(e)):rows.append((f.topic(k),m,e.stamp))
    if f.poses:
        from nav_msgs.msg import Path as PathMsg
        from conpr_player.messages import header
        t=ds.events[hi-1].stamp;rows.append((f.topic('path'),PathMsg(header=header(t,f.frames['gt_world']),poses=f.poses),t))
    return rows


class Receiver:
    def __init__(self,options):
        self.rows=[];self.lock=threading.Lock();self.subs=[];self.thread=None
        if not ROS1:
            from rclpy.node import Node
            from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
            self.node=Node('conpr_test_receiver')
            self.executor=backend.SingleThreadedExecutor();self.executor.add_node(self.node)
        for key,typ in TYPES.items():
            if key=='custom' and not options.custom:continue
            if ROS1:
                import roslib.message,rospy
                cls=roslib.message.get_message_class(typ)
            else:
                from rosidl_runtime_py.utilities import get_message
                cls=get_message(typ)
            from conpr_player.messages import NAMES
            name=NAMES[key];topic=name if name.startswith('/') else '/'+(options.namespace+'/' if options.namespace else '')+name
            def cb(msg,topic=topic,key=key):
                t=ns(msg.clock) if key=='clock' else ns(msg.transforms[0].header.stamp) if key in ('tf','static') else ns(msg.header.stamp)
                with self.lock:self.rows.extend(fingerprints(topic,msg,t))
            if ROS1:self.subs.append(rospy.Subscriber(topic,cls,cb,queue_size=10000))
            else:
                qos=QoSProfile(depth=10000,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL if key in ('static','path') else DurabilityPolicy.VOLATILE)
                self.subs.append(self.node.create_subscription(cls,topic,cb,qos))
        if not ROS1:
            self.thread=threading.Thread(target=backend.spin_executor,args=(self.executor,),daemon=True);self.thread.start()
    def clear(self):
        with self.lock:self.rows.clear()
    def close(self):
        if ROS1:
            for sub in self.subs:sub.unregister()
        else:
            self.executor.shutdown(timeout_sec=3);self.thread.join(3);self.node.destroy_node()


def wait(predicate,seconds=20):
    deadline=time.monotonic()+seconds
    while not predicate():
        if time.monotonic()>deadline:raise AssertionError('Timed out')
        time.sleep(.02)


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--output',required=True);p.add_argument('--end',type=float,default=.6);args=p.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    backend.init();results=[]
    try:
        for site,session in [('Hall','260905'),('Office','230531'),('Quad','261006')]:
            print('TEST',ROS_DISTRO,site,flush=True)
            ds=Dataset(Path(args.root)/site/session);options=Options(gt=True,gt_tf=True,custom=True,namespace='conpr_probe')
            expected=expected_messages(ds,options,args.end)
            want=collections.Counter(record for row in expected for record in fingerprints(*row))
            live_want=expected_live_records(ds,expected,args.end)
            # Independently check representative payloads against source bytes/fields.
            f=MessageFactory(ds,options)
            for sensor in ('cam','livox','imu','gt'):
                e=next(e for e in ds.events if e.sensor==sensor);data=ds.decode(e);m=dict(f.messages(e,data))
                if sensor=='cam':
                    assert bytes(m['camera'].data)==data.tobytes()
                    K=m['info'].K if ROS1 else m['info'].k
                    assert list(K)==ds.camera['camera_matrix']['data']
                if sensor=='livox':
                    import numpy as np
                    cloud=m['points'];dt=np.dtype({'names':[x.name for x in cloud.fields],'formats':['<f4']*4+['u1']*3+['<u4'],'offsets':[x.offset for x in cloud.fields],'itemsize':24})
                    pts=np.frombuffer(cloud.data,dtype=dt)
                    for name in data.dtype.names:assert np.array_equal(pts[name],data[name])
                    for i in (0,len(data)//2,len(data)-1):
                        for name in data.dtype.names:assert getattr(m['custom'].points[i],name)==data[name][i]
                if sensor=='imu':assert m['imu'].angular_velocity.x==data[4] and m['imu'].linear_acceleration.z==data[9]
                if sensor=='gt':assert m['pose'].pose.position.x==data[0]
            receiver=Receiver(options);node=PlayerNode(MessageFactory(ds,options))
            executor=backend.SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=backend.spin_executor,args=(executor,),daemon=True);thread.start()
            ctl=Playback(ds,node.emit,node.reset,end_s=args.end,on_finish=node.finish);ctl.enabled['gt']=True;node.controller=ctl
            try:
                time.sleep(1.5);node.reset(ctl.position);ctl.play();wait(lambda:ctl.state in ('FINISHED','ERROR'))
                assert ctl.state=='FINISHED',ctl.snapshot()
                wait(lambda:len(receiver.rows)>=sum(live_want.values()))
                actual=collections.Counter(receiver.rows)
                assert actual==live_want,dict(missing=list((live_want-actual).items())[:3],extra=list((actual-live_want).items())[:3])
                live_count=len(receiver.rows)
            finally:
                ctl.close();executor.shutdown(timeout_sec=3);thread.join(3);node.destroy_node();receiver.close()
            bag=out/(site+('.bag' if ROS1 else '_mcap'))
            summary=export(ds,bag,options,end_s=args.end);assert summary['complete']
            if ROS1:
                import rosbag
                with rosbag.Bag(str(bag)) as reader:
                    actual=collections.Counter(record for topic,m,t in reader.read_messages()
                                               for record in fingerprints(topic,m,t.to_nsec()))
            else:
                import rosbag2_py
                from rclpy.serialization import deserialize_message
                from rosidl_runtime_py.utilities import get_message
                reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
                types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()};actual=collections.Counter()
                while reader.has_next():
                    topic,data,t=reader.read_next()
                    actual.update(fingerprints(topic,deserialize_message(data,types[topic]),t))
                del reader
            assert actual==want,('bag roundtrip',site)
            # Late-joining latched/transient Path subscribers may receive only the final
            # path from fast bag replay; validate all sensor/clock/pose/TF samples exactly.
            receiver=Receiver(options);time.sleep(.5)
            cmd=['rosbag','play',str(bag),'--delay=0.2','--wait-for-subscribers','--rate=1'] if ROS1 else ['ros2','bag','play',str(bag),'--delay','2','--rate','1']
            try:
                proc=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=40)
                (out/(site+'_bag_play.log')).write_text(proc.stdout);assert proc.returncode==0,proc.stdout
                time.sleep(1)
                excluded={'/conpr_probe/conpr/gt/path','/tf_static'}
                if ROS1:
                    # rosbag's TimePublisher advertises /clock with queue size 1,
                    # even without --clock. Repeated identical clock values can
                    # coalesce. Require every distinct original value, in order;
                    # sensor/pose/TF multiplicity remains strictly checked below.
                    clocks=[x for x in receiver.rows if x[0]=='/clock']
                    assert set(clocks)=={x for x in want if x[0]=='/clock'},'Clock values differ'
                    assert [x[1] for x in clocks]==sorted(x[1] for x in clocks),'Clock went backwards'
                    excluded.add('/clock')
                sensor_want=collections.Counter({k:v for k,v in want.items() if k[0] not in excluded})
                sensor_got=collections.Counter(x for x in receiver.rows if x[0] not in excluded)
                assert sensor_got==sensor_want,dict(stage='bag play',missing=list((sensor_want-sensor_got).items())[:3],extra=list((sensor_got-sensor_want).items())[:3],
                    missing_by_topic=dict(collections.Counter(k[0] for k in (sensor_want-sensor_got).elements())),
                    extra_by_topic=dict(collections.Counter(k[0] for k in (sensor_got-sensor_want).elements())),
                    missing_unique_clock_stamps=sorted({k[1] for k in sensor_want if k[0]=='/clock'}-{k[1] for k in sensor_got if k[0]=='/clock'}))
                assert any(x[0]=='/tf_static' for x in receiver.rows),'Static TF missing in bag playback'
            finally:receiver.close()
            results.append(dict(site=site,session=session,live_records=live_count,bag_records=sum(want.values()),counts=summary['counts'],payloads=True,live=True,roundtrip=True,bag_play=True))
            (out/'integration.json').write_text(json.dumps(dict(ros_distro=ROS_DISTRO,passed=True,sites=results),indent=2)+'\n')
            print('PASS',site,live_count,flush=True)
    finally:backend.shutdown()

if __name__=='__main__':main()

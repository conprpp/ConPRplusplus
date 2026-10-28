"""Offline bag export using native writers and original integer nanoseconds."""
from pathlib import Path
from collections import Counter
import json
import yaml
from .environment import ROS1,ROS_DISTRO
from .messages import MessageFactory,Options,TYPES,stamp


def ros2_topic_metadata(key,topic,topic_id,options):
    """Humble uses a YAML string; Jazzy uses a vector of native QoS objects."""
    import rosbag2_py
    latched=key in ('static','path')
    reliable=latched or options.reliable
    try:metadata=rosbag2_py.TopicMetadata(id=topic_id,name=topic,type=TYPES[key],serialization_format='cdr')
    except TypeError:metadata=rosbag2_py.TopicMetadata(name=topic,type=TYPES[key],serialization_format='cdr')
    if isinstance(metadata.offered_qos_profiles,str):
        infinity={'sec':9223372036,'nsec':854775807}
        metadata.offered_qos_profiles=yaml.safe_dump([dict(
            history=1,depth=1 if latched else 256,reliability=1 if reliable else 2,
            durability=1 if latched else 2,deadline=infinity,lifespan=infinity,
            liveliness=1,liveliness_lease_duration=infinity,avoid_ros_namespace_conventions=False)])
    else:
        qos=rosbag2_py._storage.QoS(1 if latched else 256)
        qos.reliable() if reliable else qos.best_effort()
        if latched:qos.transient_local()
        metadata.offered_qos_profiles=[qos]
        if hasattr(metadata,'id'):metadata.id=topic_id
    return metadata


class Writer:
    def __init__(self,path,options,storage):
        self.options=options;self.created=set();self.ros1=ROS1
        if ROS1:
            import rosbag
            if storage not in (None,'bag'):raise ValueError('Noetic export uses ROS 1 .bag, not rosbag2 storage.')
            if path.suffix!='.bag':raise ValueError('ROS 1 output must end in .bag')
            self.writer=rosbag.Bag(str(path),'w')
        else:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            self.serialize=serialize_message
            if storage not in (None,'mcap','sqlite3'):raise ValueError('ROS 2 storage must be mcap or sqlite3')
            self.writer=rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(path),storage_id=storage or 'mcap'),rosbag2_py.ConverterOptions('',''))
    def write(self,key,topic,msg,t):
        if self.ros1:
            connection=dict(topic=topic,type=msg._type,md5sum=msg._md5sum,message_definition=msg._full_text,
                            callerid='/conpr_file_player',latching='1' if key in ('static','path') else '0')
            self.writer.write(topic,msg,t=stamp(t),connection_header=connection)
        else:
            if key not in self.created:
                self.writer.create_topic(ros2_topic_metadata(key,topic,len(self.created),self.options));self.created.add(key)
            self.writer.write(topic,self.serialize(msg),int(t))
    def close(self):
        if self.ros1:self.writer.close()
        # Humble finalizes metadata in the destructor; Jazzy also supports close().
        elif hasattr(self.writer,'close'):self.writer.close()
        self.writer=None


def export(ds,destination,options=None,start_s=0,end_s=None,progress=None,cancel=None,skip_intervals=(),storage=None):
    path=Path(destination).expanduser().resolve()
    sidecar=Path(str(path)+'.conpr_export.json') if ROS1 else path/'conpr_export.json'
    if path.exists() or sidecar.exists():raise FileExistsError(f'Refusing to overwrite {path}')
    path.parent.mkdir(parents=True,exist_ok=True)
    factory=MessageFactory(ds,options or Options());lo,hi=ds.bounds(start_s,end_s)
    if lo>=hi:raise ValueError('Empty export range')
    writer=Writer(path,factory.options,storage);counts=Counter();complete=False;err=None
    def write(key,msg,t):
        writer.write(key,factory.topic(key),msg,t);counts[key]+=1
    try:
        for key,msg in factory.reset(ds.events[lo].stamp):write(key,msg,ds.events[lo].stamp)
        for i in range(lo,hi):
            if cancel and cancel.is_set():raise InterruptedError('Export cancelled; partial bag retained with complete=false')
            e=ds.events[i]
            if any(a<=(e.stamp-ds.start)/1e9<b for a,b in skip_intervals):continue
            active={'cam':factory.options.camera,'livox':factory.options.lidar,'imu':factory.options.imu,'gt':factory.options.gt}
            if not active[e.sensor]:continue
            for key,msg in factory.messages(e,ds.decode(e)):write(key,msg,e.stamp)
            if progress and (i-lo)%1000==0:progress(i-lo,hi-lo,dict(counts))
        if factory.poses:
            from nav_msgs.msg import Path as PathMessage
            from .messages import header
            t=ds.events[hi-1].stamp
            write('path',PathMessage(header=header(t,factory.frames['gt_world']),poses=factory.poses),t)
        complete=True
    except BaseException as exc:err=exc
    finally:
        try:writer.close()
        except Exception as close_error:
            complete=False
            if err is None:err=close_error
        summary=dict(complete=complete,ros_distro=ROS_DISTRO,storage='bag' if ROS1 else storage or 'mcap',
            source=str(ds.path),start_s=start_s,end_s=end_s,counts=dict(counts),
            clock='Original integer nanoseconds; no rebasing',error=None if err is None else str(err),skip_intervals=list(skip_intervals))
        sidecar.write_text(json.dumps(summary,indent=2)+'\n')
    if err:raise err
    if progress:progress(hi-lo,hi-lo,dict(counts))
    return summary

"""Shared ROS message construction. Sensor values and nanosecond stamps are unchanged."""
from dataclasses import dataclass
import copy
import numpy as np
from .environment import ROS1
if ROS1:
    from rospy import Time
else:
    from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image,CameraInfo,Imu,PointCloud2,PointField
from std_msgs.msg import Header,String
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

@dataclass
class Options:
    camera: bool=True
    lidar: bool=True
    imu: bool=True
    gt: bool=False
    gt_tf: bool=False
    custom: bool=False
    frame_prefix: str='conpr'
    namespace: str=''
    reliable: bool=True

TYPES={'clock':'rosgraph_msgs/msg/Clock','points':'sensor_msgs/msg/PointCloud2','camera':'sensor_msgs/msg/Image',
       'info':'sensor_msgs/msg/CameraInfo','imu':'sensor_msgs/msg/Imu','pose':'geometry_msgs/msg/PoseStamped',
       'path':'nav_msgs/msg/Path','tf':'tf2_msgs/msg/TFMessage','static':'tf2_msgs/msg/TFMessage','custom':'livox_interfaces/msg/CustomMsg'}
if ROS1:
    TYPES={k:v.replace('/msg/','/') for k,v in TYPES.items()}
    TYPES['custom']='livox_ros_driver/CustomMsg'
NAMES={'clock':'/clock','points':'livox/points','camera':'camera/color/image_raw','info':'camera/color/camera_info',
       'imu':'imu','pose':'conpr/gt/pose','path':'conpr/gt/path','tf':'/tf','static':'/tf_static','custom':'livox/lidar'}

def stamp(t):
    sec,ns=divmod(int(t),1_000_000_000)
    return Time(sec,ns) if ROS1 else Time(sec=sec,nanosec=ns)
def header(t,frame):return Header(stamp=stamp(t),frame_id=frame)
def quaternion(R):
    # Symmetric eigenproblem avoids singular trace branches near 180 degrees.
    r=np.asarray(R)
    K=np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
                [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
                [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
                [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])
    q=np.linalg.eigh(K)[1][:,-1]
    return q if q[3]>=0 else -q

def fill_quaternion(q,values):q.x,q.y,q.z,q.w=map(float,values)
def fill_vector(v,values):v.x,v.y,v.z=map(float,values)
def transform(t,parent,child,T):
    m=TransformStamped(header=header(t,parent),child_frame_id=child)
    fill_vector(m.transform.translation,T[:3,3]);fill_quaternion(m.transform.rotation,quaternion(T[:3,:3]));return m

class MessageFactory:
    def __init__(self,ds,options=None):
        self.ds=ds;self.options=options or Options();self.poses=[];self.last_path_stamp=None
        p=self.options.frame_prefix.strip('/')
        self.frames={s:f'{p}_{s}' for s in ('imu','lidar','camera_optical','gt_world')}
        self.CustomMsg=self.CustomPoint=None
        if self.options.custom:self.require_custom()
        c=ds.camera
        self.info=CameraInfo(height=c['image_height'],width=c['image_width'],distortion_model=c['distortion_model'])
        for field,key in [('d','distortion_coefficients'),('k','camera_matrix'),('r','rectification_matrix'),('p','projection_matrix')]:
            setattr(self.info,field.upper() if ROS1 else field,list(map(float,c[key]['data'])))
    def require_custom(self):
        if self.CustomMsg is None:
            if ROS1:
                from livox_ros_driver.msg import CustomMsg,CustomPoint
            else:
                from livox_interfaces.msg import CustomMsg,CustomPoint
            self.CustomMsg,self.CustomPoint=CustomMsg,CustomPoint
    def topic(self,key):
        name=NAMES[key]
        if name.startswith('/'):return name
        return '/'+('/'.join(x.strip('/') for x in (self.options.namespace,name) if x.strip('/')))
    def static(self,t):
        # T_parent_child; camera->IMU = LiDAR->IMU @ camera->LiDAR.
        TiL=self.ds.extrinsics['T_imu_lidar'];TcL=self.ds.extrinsics['T_camera_lidar']
        return TFMessage(transforms=[transform(t,self.frames['imu'],self.frames['lidar'],TiL),
                                     transform(t,self.frames['imu'],self.frames['camera_optical'],TiL@np.linalg.inv(TcL))])
    def reset(self,t):
        self.poses=[];self.last_path_stamp=None
        return [('clock',Clock(clock=stamp(t))),('static',self.static(t)),('path',Path(header=header(t,self.frames['gt_world'])))]
    def messages(self,event,data,include_clock=True):
        t,s=event.stamp,event.sensor;o=self.options
        result=[('clock',Clock(clock=stamp(t)))] if include_clock else []
        if s=='cam' and o.camera:
            m=Image(header=header(t,self.frames['camera_optical']),height=data.shape[0],width=data.shape[1],encoding='rgb8',is_bigendian=0,step=data.shape[1]*3,data=data.tobytes())
            info=copy.deepcopy(self.info);info.header=m.header
            result.extend([('camera',m),('info',info)])
        elif s=='imu' and o.imu:
            m=Imu(header=header(t,self.frames['imu']));fill_quaternion(m.orientation,data[:4]);fill_vector(m.angular_velocity,data[4:7]);fill_vector(m.linear_acceleration,data[7:10])
            result.append(('imu',m))
        elif s=='livox' and o.lidar:
            fields=[PointField(name=n,offset=offset,datatype=dt,count=1) for n,offset,dt in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('reflectivity',16,2),('tag',17,2),('line',18,2),('offset_time',20,6)]]
            dtype=np.dtype({'names':[f.name for f in fields],'formats':['<f4']*4+['u1']*3+['<u4'],'offsets':[f.offset for f in fields],'itemsize':24})
            pts=np.zeros(len(data),dtype=dtype)
            for n in data.dtype.names:pts[n]=data[n]
            pts['intensity']=data['reflectivity']
            result.append(('points',PointCloud2(header=header(t,self.frames['lidar']),height=1,width=len(data),fields=fields,is_bigendian=False,point_step=24,row_step=24*len(data),data=pts.tobytes(),is_dense=True)))
            if o.custom:
                self.require_custom();m=self.CustomMsg(header=header(t,self.frames['lidar']),timebase=t,point_num=len(data),lidar_id=0)
                m.points=[self.CustomPoint(offset_time=int(p['offset_time']),x=float(p['x']),y=float(p['y']),z=float(p['z']),reflectivity=int(p['reflectivity']),tag=int(p['tag']),line=int(p['line'])) for p in data]
                result.append(('custom',m))
        elif s=='gt' and o.gt:
            m=PoseStamped(header=header(t,self.frames['gt_world']));fill_vector(m.pose.position,data[:3]);fill_quaternion(m.pose.orientation,data[3:7]);result.append(('pose',m));self.poses.append(m)
            if self.last_path_stamp is None or t-self.last_path_stamp>=1_000_000_000:
                result.append(('path',Path(header=m.header,poses=list(self.poses))));self.last_path_stamp=t
            if o.gt_tf:
                tf=TransformStamped(header=m.header,child_frame_id=self.frames['imu']);fill_vector(tf.transform.translation,data[:3]);fill_quaternion(tf.transform.rotation,data[3:7]);result.append(('tf',TFMessage(transforms=[tf])))
        return result


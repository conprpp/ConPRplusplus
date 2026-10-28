"""Read only the public ConPRplusplus layout; no research-workspace imports."""
from dataclasses import dataclass
from pathlib import Path
import csv
import numpy as np
from PIL import Image
import yaml

IMU_HEADER = 'timestamp_ns,qx,qy,qz,qw,wx,wy,wz,ax,ay,az'.split(',')
GT_HEADER = 'timestamp_ns,x,y,z,qx,qy,qz,qw'.split(',')
POINT_DTYPE = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('tag','u1'),('line','u1'),('offset_time','<u4')], align=False)

@dataclass(frozen=True, order=True)
class Event:
    stamp: int
    sensor: str

def read_numeric(path, fields):
    with Path(path).open(newline='') as f:
        reader = csv.reader(f)
        if next(reader, None) != fields:
            raise ValueError(f'{path}: expected header {fields}')
        rows = list(reader)
    if not rows or any(len(row) != len(fields) for row in rows):
        raise ValueError(f'{path}: empty or malformed data rows')
    ts = np.array([int(r[0]) for r in rows], dtype=np.int64)
    values = np.array([[float(v) for v in r[1:]] for r in rows], dtype=np.float64)
    if np.any(np.diff(ts) <= 0) or not np.isfinite(values).all():
        raise ValueError(f'{path}: timestamps not increasing or nonfinite values')
    return ts, values

class Dataset:
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        self.site = self.path.parent.name
        self.sequence = self.path.name
        with (self.path/'stamps.csv').open(newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames != ['timestamp_ns','sensor']:
                raise ValueError('stamps.csv requires timestamp_ns,sensor header')
            self.sensor_events = [Event(int(r['timestamp_ns']), r['sensor']) for r in reader]
        if not self.sensor_events or self.sensor_events != sorted(self.sensor_events):
            raise ValueError('Empty or unordered stamps.csv')
        if len(set(self.sensor_events)) != len(self.sensor_events):
            raise ValueError('Duplicate events')
        if any(e.sensor not in ('cam','livox','imu') for e in self.sensor_events):
            raise ValueError('Unknown sensor in stamps.csv')
        self.start = self.sensor_events[0].stamp
        self.end = self.sensor_events[-1].stamp
        self.duration = (self.end-self.start)/1e9
        self.imu_ts, self.imu_values = read_numeric(self.path/'IMU/imu.csv', IMU_HEADER)
        if [e.stamp for e in self.sensor_events if e.sensor == 'imu'] != self.imu_ts.tolist():
            raise ValueError('IMU rows and stamps.csv differ')
        self.imu = dict(zip(self.imu_ts.tolist(),self.imu_values))
        gt_path = self.path/'GT_pose/poses.csv'
        if gt_path.exists():
            self.gt_ts, self.gt_values = read_numeric(gt_path, GT_HEADER)
            if np.any(np.abs(np.linalg.norm(self.gt_values[:,3:7],axis=1)-1)>1e-4):
                raise ValueError('GT quaternion is not unit length')
        else:
            self.gt_ts, self.gt_values = np.array([],dtype=np.int64),np.empty((0,7))
        self.gt = dict(zip(self.gt_ts.tolist(),self.gt_values))
        self.events = sorted(self.sensor_events+[Event(int(t),'gt') for t in self.gt_ts if self.start <= t <= self.end])
        self.times = np.array([e.stamp for e in self.events],dtype=np.int64)
        self.lidar_indices = np.array([i for i,e in enumerate(self.events) if e.sensor=='livox'])
        self.camera = yaml.safe_load((self.path.parent/'Calibration/camera.yaml').read_text())
        self.extrinsics = yaml.safe_load((self.path.parent/'Calibration/extrinsics.yaml').read_text())
        self._check_calibration()
        self.counts = {s:sum(e.sensor==s for e in self.sensor_events) for s in ('cam','livox','imu')}
        for sensor,folder,suffix in [('cam','Camera','.png'),('livox','LiDAR','.bin')]:
            expected = {str(e.stamp)+suffix for e in self.sensor_events if e.sensor==sensor}
            actual = {p.name for p in (self.path/folder).glob('*'+suffix)}
            if actual != expected:
                raise ValueError(f'{folder} file/index mismatch: missing={len(expected-actual)}, extra={len(actual-expected)}')
        # Decode representative samples now, remaining files validated while read.
        for sensor in ('cam','livox'):
            self.decode(next(e for e in self.sensor_events if e.sensor==sensor))

    def _check_calibration(self):
        c=self.camera
        if c['image_width']<=0 or c['image_height']<=0:
            raise ValueError('Invalid image resolution')
        for key,shape in [('camera_matrix',(3,3)),('rectification_matrix',(3,3)),('projection_matrix',(3,4))]:
            m=c[key]
            if (m['rows'],m['cols'])!=shape or len(m['data'])!=shape[0]*shape[1] or not np.isfinite(m['data']).all():
                raise ValueError(f'Invalid {key}')
        if c['camera_matrix']['data'][0]<=0 or c['camera_matrix']['data'][4]<=0:
            raise ValueError('Invalid focal length')
        d=c['distortion_coefficients']['data']
        if not np.isfinite(d).all() or (c['distortion_model'],len(d)) not in [('plumb_bob',5),('equidistant',4)]:
            raise ValueError('Unsupported distortion schema')
        for name in ('T_imu_lidar','T_camera_lidar'):
            T=np.asarray(self.extrinsics[name],dtype=float)
            if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):
                raise ValueError(f'Invalid transform {name}')
            if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-4) or abs(np.linalg.det(T[:3,:3])-1)>1e-4:
                raise ValueError(f'Invalid rotation {name}')
            self.extrinsics[name]=T

    def decode(self,event):
        t,s=event.stamp,event.sensor
        if s=='imu': return self.imu[t]
        if s=='gt': return self.gt[t]
        if s=='cam':
            with Image.open(self.path/'Camera'/f'{t}.png') as im:
                im.load()
                if im.size!=(self.camera['image_width'],self.camera['image_height']):
                    raise ValueError(f'Image/calibration resolution mismatch: {t}')
                return np.array(im.convert('RGB'),copy=True)
        data=(self.path/'LiDAR'/f'{t}.bin').read_bytes()
        if not data or len(data)%19: raise ValueError(f'Empty/truncated LiDAR: {t}')
        pts=np.frombuffer(data,dtype=POINT_DTYPE)
        if not all(np.isfinite(pts[n]).all() for n in ('x','y','z')) or int(pts['offset_time'].max())>110_000_000:
            raise ValueError(f'Invalid LiDAR geometry/point time: {t}')
        return pts

    def validate(self, full=False, progress=None):
        checked=0
        if full:
            for e in self.sensor_events:
                if e.sensor in ('cam','livox'):
                    self.decode(e);checked+=1
                    if progress and checked%500==0:progress(checked)
        return dict(site=self.site,sequence=self.sequence,counts=self.counts,gt=len(self.gt),duration_s=self.duration,full=full,decoded_files=checked,passed=True)

    def bounds(self,start_s=0,end_s=None):
        if start_s<0 or (end_s is not None and (end_s<=start_s or end_s>self.duration+1e-6)) or start_s>self.duration:
            raise ValueError('Invalid start/end range')
        lo=int(np.searchsorted(self.times,self.start+round(start_s*1e9),'left'))
        hi=len(self.events) if end_s is None else int(np.searchsorted(self.times,self.start+round(end_s*1e9),'right'))
        return lo,hi

def discover(root):
    if not root or not str(root).strip():return []
    root=Path(root).expanduser().resolve()
    if (root/'stamps.csv').exists():return [root]
    if (root/'Calibration').is_dir():return sorted(p.parent for p in root.glob('*/stamps.csv'))
    return sorted(p.parent for p in root.glob('*/*/stamps.csv'))

def load_intervals(path,duration):
    with Path(path).open() as f:
        rd=csv.DictReader(f)
        if rd.fieldnames!=['start_s','end_s']:raise ValueError('Skip CSV header must be start_s,end_s')
        rows=[(float(r['start_s']),float(r['end_s'])) for r in rd]
    if any(not 0<=a<b<=duration for a,b in rows):raise ValueError('Invalid skip interval')
    rows.sort()
    if any(a[1]>b[0] for a,b in zip(rows,rows[1:])):raise ValueError('Skip intervals overlap')
    return rows

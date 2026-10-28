"""Generate a tiny artificial dataset for CI; never presented as ConPR++ observations."""
import csv,sys
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from conpr_player.dataset import POINT_DTYPE,IMU_HEADER,GT_HEADER
root=Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True)
for site,session,fps in [('Hall','260905',10),('Office','230531',30),('Quad','261006',10)]:
    seq=root/site/session;cal=seq.parent/'Calibration';cal.mkdir(parents=True,exist_ok=True)
    for d in ['Camera','LiDAR','IMU','GT_pose']:(seq/d).mkdir(parents=True,exist_ok=True)
    K=[50.,0.,32.,0.,50.,24.,0.,0.,1.]
    camera=dict(image_width=64,image_height=48,distortion_model='plumb_bob',camera_matrix=dict(rows=3,cols=3,data=K),distortion_coefficients=dict(rows=1,cols=5,data=[0.]*5),rectification_matrix=dict(rows=3,cols=3,data=np.eye(3).flatten().tolist()),projection_matrix=dict(rows=3,cols=4,data=[50.,0.,32.,0.,0.,50.,24.,0.,0.,0.,1.,0.]))
    (cal/'camera.yaml').write_text(yaml.safe_dump(camera));(cal/'extrinsics.yaml').write_text(yaml.safe_dump(dict(T_imu_lidar=np.eye(4).tolist(),T_camera_lidar=np.eye(4).tolist())))
    t0=1_700_000_000_000_000_000;events=[];imus=[];gts=[]
    for i in range(751):
        t=t0+i*4_000_000;events.append((t,'imu'));imus.append([t,0.,0.,0.,1.,.1,.2,.3,0.,0.,9.81])
    for i in range(31):
        t=t0+i*100_000_000;events.append((t,'livox'));gts.append([t,i*.01,0.,0.,0.,0.,0.,1.])
        pts=np.zeros(80,dtype=POINT_DTYPE);pts['x']=np.linspace(1,5,80);pts['y']=np.linspace(-1,1,80);pts['z']=.1;pts['reflectivity']=np.arange(80);pts['line']=1;pts['offset_time']=np.arange(80)*1000
        (seq/'LiDAR'/f'{t}.bin').write_bytes(pts.tobytes())
    for i in range(3*fps+1):
        t=t0+round(i*1e9/fps);events.append((t,'cam'));Image.new('RGB',(64,48),(i%256,80,120)).save(seq/'Camera'/f'{t}.png')
    for path,header,rows in [(seq/'stamps.csv',['timestamp_ns','sensor'],sorted(events)),(seq/'IMU/imu.csv',IMU_HEADER,imus),(seq/'GT_pose/poses.csv',GT_HEADER,gts)]:
        with path.open('w',newline='') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
print(root)

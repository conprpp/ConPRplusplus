"""Native GUI/GT integration probe; requires a ROS1 master for Noetic and X11."""
import argparse,json,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import yaml
from PyQt5 import QtWidgets as W,QtCore as C
from conpr_player import backend
from conpr_player.environment import ROS_DISTRO
from conpr_player.gui import Window

p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
backend.init();app=W.QApplication([])
args=SimpleNamespace(root=a.root,dataset=None,rate=1.,gt=False,gt_tf=False,custom=False,namespace='follow_test',best_effort=False,skip_invalid=False,start=0.,end=10.)
w=Window(args);w.show();errors=[];stage=0;deadline=time.monotonic()+60;record={}
def fail(text):raise RuntimeError(text)
w.failed=fail
class FakeProcess:
    def __init__(self):self.done=False
    def poll(self):return 0 if self.done else None
    def terminate(self):self.done=True
calls=[]
def capture(cmd,**kwargs):calls.append(cmd);return FakeProcess()
try:
    # Test view configuration and automatic restart without launching extra RViz windows.
    with patch('conpr_player.gui.subprocess.Popen',side_effect=capture):
        w.open_rviz();original=w.rviz_process
        w.follow_gt.setChecked(True)
        assert original.done and len(calls)==2
        config=yaml.safe_load(Path(calls[-1][2]).read_text())['Visualization Manager']
        assert config['Global Options']['Fixed Frame']=='conpr_gt_world'
        assert config['Views']['Current']['Target Frame']=='conpr_lidar'
        path=next(d for d in config['Displays'] if d['Class'].endswith('/Path'))
        cloud=next(d for d in config['Displays'] if d['Class'].endswith('/PointCloud2'))
        rgb=next(d for d in config['Displays'] if d['Class'].endswith('/Image'))
        if ROS_DISTRO=='noetic':
            assert rgb['Image Topic']=='/follow_test/camera/color/image_raw' and rgb['Transport Hint']=='raw'
        else:assert rgb['Topic']['Value']=='/follow_test/camera/color/image_raw'
        assert path['Enabled'] and cloud['Decay Time']==0
        topic=path['Topic'];assert (topic if isinstance(topic,str) else topic['Value'])=='/follow_test/conpr/gt/path'
        assert all(w.checks[n].isChecked() and not w.checks[n].isEnabled() for n in ('gt','gt_tf'))
        w.follow_gt.setChecked(False)
        assert all(not w.checks[n].isChecked() and w.checks[n].isEnabled() for n in ('gt','gt_tf'))
        w.rviz_process.terminate();w.rviz_process=None
    w.follow_gt.setChecked(True)
    i=w.sessions.findData(str((Path(a.root)/'Hall/260905').resolve()));assert i>=0
    w.sessions.setCurrentIndex(i);w.load()
    def tick():
        global stage
        try:
            assert time.monotonic()<deadline,('timeout',stage)
            if stage==0 and w.controller:
                assert w.node.factory.options.gt and w.node.factory.options.gt_tf
                w.open_rviz();w.control('play');stage=1
            elif stage==1 and w.node.message_counts.get('tf',0)>=10:
                w.control('pause');assert w.rviz_process.poll() is None
                assert w.node.message_counts['points']>0 and w.node.message_counts['path']>0
                record.update(passed=True,ros_distro=ROS_DISTRO,counts=dict(w.node.message_counts),current_scan_only=True,follow_frame='conpr_lidar',fixed_frame='conpr_gt_world',window_exposed=w.windowHandle().isExposed())
                w.grab().save(str(out/'gui.png'))
                # External parameter disable cancels follow mode without re-enabling GT.
                w.node.set_parameters([backend.Parameter('gt_tf',value=False)])
                w.rviz_process.terminate();w.rviz_process.wait(timeout=10);w.rviz_process=None
                w.refresh();assert not w.follow_gt.isChecked() and not w.node.factory.options.gt_tf
                w.close();app.quit()
        except Exception as e:errors.append(repr(e));w.close();app.quit()
    timer=C.QTimer();timer.timeout.connect(tick);timer.start(100);app.exec_()
finally:
    if w.rviz_process and w.rviz_process.poll() is None:w.rviz_process.terminate();w.rviz_process.wait(timeout=10)
    w.dispose();backend.shutdown()
if errors:raise RuntimeError(errors)
(out/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(record)

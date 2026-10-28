"""Capture actual RViz RGB/cloud/TF status, with follow mode off and on.

Requires a desktop (or isolated Xvfb), xdotool, and a ROS1 master for Noetic.
Screenshots require visual inspection: process success alone is not a rendering test.
"""
import argparse,json,subprocess,time
from pathlib import Path
from types import SimpleNamespace
from PyQt5 import QtWidgets as W
from conpr_player import backend
from conpr_player.environment import ROS_DISTRO,ROS1
from conpr_player.gui import Window

p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
backend.init();app=W.QApplication([])
args=SimpleNamespace(root=a.root,dataset=None,rate=1.,gt=False,gt_tf=False,custom=False,namespace='render_test',best_effort=False,skip_invalid=False,start=0.,end=30.)
w=Window(args);w.show();record={};errors=[]
def fail(text):raise RuntimeError(text)
w.failed=fail

def wait(predicate,timeout=30):
    end=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>end:raise RuntimeError('wait timed out')
        app.processEvents();time.sleep(.03)
def tick(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:app.processEvents();time.sleep(.03)
def screenshot(name):
    ids=subprocess.check_output(['xdotool','search','--onlyvisible','--pid',str(w.rviz_process.pid)],text=True).split()
    wid=next(x for x in ids if 'RViz' in subprocess.check_output(['xdotool','getwindowname',x],text=True))
    subprocess.run(['xdotool','windowraise',wid],check=True)
    # Expand Livox and its Status. Default configs have the same property-tree rows.
    for x,y in ((10,327),(30,347)):
        subprocess.run(['xdotool','mousemove','--window',wid,str(x),str(y),'click','1'],check=True);tick(.3)
    pix=app.primaryScreen().grabWindow(int(wid));assert not pix.isNull()
    pix.save(str(out/(name+'.png')))

def subscriptions():
    if ROS1:
        import rosgraph
        s=dict(rosgraph.Master('/render_audit').getSystemState()[1])
        return {k:len(s.get('/render_test/'+k,[])) for k in ('livox/points','camera/color/image_raw')}
    return {k:w.node.count_subscribers('/render_test/'+k) for k in ('livox/points','camera/color/image_raw')}
try:
    i=w.sessions.findData(str((Path(a.root)/'Hall/260905').resolve()));assert i>=0
    w.sessions.setCurrentIndex(i);w.load();wait(lambda:w.controller is not None)
    for follow in (False,True):
        w.follow_gt.setChecked(follow)
        wait(lambda:not w.rviz_restarting)
        w.open_rviz();w.control('play');tick(6)
        assert w.rviz_process.poll() is None
        sub=subscriptions();assert all(sub.values()),sub
        w.control('pause');tick(.3)
        screenshot('follow_on' if follow else 'follow_off')
        record['follow_on' if follow else 'follow_off']={'subscriptions':sub,'messages':dict(w.node.message_counts)}
    record.update(ros_distro=ROS_DISTRO,transport_checks_passed=True,rendering_verdict='pending screenshot review')
finally:
    if w.rviz_process and w.rviz_process.poll() is None:w.rviz_process.terminate();w.rviz_process.wait(timeout=10)
    w.close();w.dispose();backend.shutdown()
(out/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(record)

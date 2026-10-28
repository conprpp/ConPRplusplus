"""Exercise actual Qt widgets on an X display and save an inspection screenshot."""
import argparse,json,time
from pathlib import Path
from types import SimpleNamespace
from conpr_player import backend
from conpr_player.environment import ROS_DISTRO
from PyQt5 import QtWidgets as W,QtCore as C
from conpr_player.gui import Window
parser=argparse.ArgumentParser();parser.add_argument('--root',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
root=Path(args.root).resolve();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
a=SimpleNamespace(root=str(root),dataset=str(root/'Hall/260905'),rate=1.,gt=False,gt_tf=False,custom=True,namespace='gui_audit',best_effort=False,skip_invalid=False,start=0.,end=None)
backend.init();app=W.QApplication([]);win=Window(a);win.show();stage=0;deadline=time.monotonic()+90;record={};exit_error=[];last=time.monotonic()
def tick():
    global stage,last
    try:
        assert time.monotonic()<deadline,('GUI timeout',stage)
        if stage==0 and win.controller:
            win.buttons['play'].click();stage=1
        elif stage==1 and win.controller.counts['cam']>=4:
            win.buttons['pause'].click();assert win.controller.state=='PAUSED';record['pause']=dict(win.controller.counts);last=time.monotonic();stage=2
        elif stage==2 and time.monotonic()-last>.2:
            assert dict(win.controller.counts)==record['pause'];win.checks['gt'].click();win.checks['gt_tf'].click();win.slider.setValue(5000);win.seek_slider();assert win.controller.epoch>=1
            win.buttons['step'].click();stage=3
        elif stage==3 and win.controller.state=='PAUSED':
            record['seek_step']=win.controller.snapshot();win.rate.setValue(2.);win.loop.click();assert win.controller.loop;win.buttons['play'].click();last=time.monotonic();stage=4
        elif stage==4 and time.monotonic()-last>1.5:
            win.buttons['pause'].click();win.refresh();win.grab().save(str(out/'gui_preview.png'));record['screenshot']='gui_preview.png';win.buttons['stop'].click();assert win.controller.state=='STOPPED';record['passed']=True;record['ros_distro']=ROS_DISTRO;record['qt_platform']=app.platformName();record['window_exposed']=win.windowHandle().isExposed()
            (out/'gui_probe.json').write_text(json.dumps(record,indent=2)+'\n');win.close();app.quit()
    except Exception as exc:
        exit_error.append(repr(exc));win.close();app.quit()
timer=C.QTimer();timer.timeout.connect(tick);timer.start(50)
app.exec_();win.dispose();backend.shutdown()
if exit_error:raise RuntimeError(exit_error)
print('GUI PASS',record,flush=True)

"""Qt GUI; controls and ROS services share exactly one playback controller."""
import copy,json,subprocess,threading,time,os,tempfile
import yaml
from pathlib import Path
import numpy as np
from PyQt5 import QtWidgets as W,QtCore as C,QtGui as G
from .backend import SingleThreadedExecutor
from .backend import Parameter
from .environment import ROS1,DISPLAY_NAME
from .dataset import Dataset,discover,load_intervals
from .controller import Playback
from .ros import MessageFactory,PlayerNode,Options,spin_executor

class Signals(C.QObject):
    loaded=C.pyqtSignal(object)
    failed=C.pyqtSignal(str)
    export_update=C.pyqtSignal(str)
    export_done=C.pyqtSignal(str)

class CloudView(W.QWidget):
    def __init__(self):
        super().__init__();self.points=None;self.setMinimumSize(430,300)
    def paintEvent(self,event):
        p=G.QPainter(self);p.fillRect(self.rect(),G.QColor('#101c2b'));p.setRenderHint(G.QPainter.Antialiasing)
        p.setPen(G.QColor('#546b86'));cx,cy=self.width()/2,self.height()*.88
        p.drawLine(int(cx),20,int(cx),int(cy));p.drawLine(20,int(cy),self.width()-20,int(cy))
        p.setPen(G.QColor('#dce7f2'));p.drawText(15,25,'LiDAR · top view · +x forward ↑ · ±20 m lateral / 40 m forward')
        if self.points is None:return
        pts=self.points[::max(1,len(self.points)//5000)]
        scale=min(self.width()/40,(self.height()-45)/40)
        p.setPen(G.QPen(G.QColor('#53d4cc'),1))
        for a in pts:
            x=cx-float(a['y'])*scale;y=cy-float(a['x'])*scale
            if 0<x<self.width() and 35<y<self.height():p.drawPoint(C.QPointF(x,y))

class Window(W.QMainWindow):
    def __init__(self,args):
        super().__init__();self.args=args;self.ds=None;self.controller=None;self.node=None;self.executor=None;self.spin_thread=None;self.closing=False
        self.latest_image=None;self.latest_points=None;self.last_image=None;self.export_thread=None;self.export_cancel=threading.Event();self.rviz_process=None
        self.rviz_configs=tempfile.TemporaryDirectory(prefix='conpr_rviz_');self.rviz_restarting=False;self.follow_previous=None
        self.signals=Signals();self.signals.loaded.connect(self.loaded);self.signals.failed.connect(self.failed);self.signals.export_update.connect(self.log);self.signals.export_done.connect(self.export_done)
        self.setWindowTitle('ConPR++ · '+DISPLAY_NAME+' File Player');self.resize(1180,820)
        central=W.QWidget();self.setCentralWidget(central);layout=W.QVBoxLayout(central)
        title=W.QLabel('ConPR++   /   Multi-session dataset player');title.setStyleSheet('font-size:23px;font-weight:600;color:#162a42;padding:6px');layout.addWidget(title)
        row=W.QHBoxLayout();self.root=W.QLineEdit(str(args.root or (Path(args.dataset).parent.parent if args.dataset else '')));row.addWidget(W.QLabel('Dataset root'));row.addWidget(self.root)
        self.browse_button=W.QPushButton('Browse…');self.browse_button.clicked.connect(self.browse);row.addWidget(self.browse_button)
        self.scan_button=W.QPushButton('Scan');self.scan_button.clicked.connect(self.scan);row.addWidget(self.scan_button);layout.addLayout(row)
        row=W.QHBoxLayout();self.sessions=W.QComboBox();row.addWidget(self.sessions,1);self.load_button=W.QPushButton('Load sequence');self.load_button.clicked.connect(self.load);row.addWidget(self.load_button)
        self.start=W.QDoubleSpinBox();self.start.setRange(0,1e5);self.start.setDecimals(3);self.end=W.QDoubleSpinBox();self.end.setRange(0,1e5);self.end.setDecimals(3);self.end.setSpecialValueText('End of sequence');self.start.setValue(args.start);self.end.setValue(args.end or 0)
        row.addWidget(W.QLabel('Start (s)'));row.addWidget(self.start);row.addWidget(W.QLabel('End (s)'));row.addWidget(self.end);layout.addLayout(row)
        self.details=W.QLabel('Select a ConPRplusplus root or a sequence directory.');layout.addWidget(self.details)
        preview=W.QHBoxLayout();self.image=W.QLabel('RGB preview');self.image.setMinimumSize(430,300);self.image.setAlignment(C.Qt.AlignCenter);self.image.setStyleSheet('background:#101c2b;color:#dce7f2');preview.addWidget(self.image,1)
        self.cloud=CloudView();preview.addWidget(self.cloud,1);layout.addLayout(preview,1)
        self.slider=W.QSlider(C.Qt.Horizontal);self.slider.setRange(0,10000);self.slider.sliderReleased.connect(self.seek_slider);layout.addWidget(self.slider)
        self.status=W.QLabel('READY');layout.addWidget(self.status)
        row=W.QHBoxLayout();self.buttons={}
        for label,action in [('Play','play'),('Pause','pause'),('Stop','stop'),('Restart','restart'),('Step LiDAR','step')]:
            btn=W.QPushButton(label);btn.clicked.connect(lambda checked=False,a=action:self.control(a));row.addWidget(btn);self.buttons[action]=btn
        self.rate=W.QDoubleSpinBox();self.rate.setRange(.05,100);self.rate.setSingleStep(.25);self.rate.setValue(args.rate);self.rate.setSuffix(' ×');self.rate.valueChanged.connect(self.change_rate);row.addWidget(self.rate)
        self.loop=W.QCheckBox('Loop');self.loop.setChecked(getattr(args,'loop',False));self.loop.toggled.connect(self.change_loop);row.addWidget(self.loop);layout.addLayout(row)
        row=W.QHBoxLayout();self.checks={}
        for name,label,default in [('camera','RGB',not getattr(args,'no_camera',False)),('lidar','LiDAR',not getattr(args,'no_lidar',False)),('imu','IMU',not getattr(args,'no_imu',False)),('gt','GT pose/path',args.gt),('gt_tf','GT world TF',args.gt_tf),('custom','Livox CustomMsg',args.custom)]:
            ch=W.QCheckBox(label);ch.setChecked(default);ch.toggled.connect(lambda v,n=name:self.toggle(n,v));row.addWidget(ch);self.checks[name]=ch
        layout.addLayout(row);row=W.QHBoxLayout()
        self.follow_gt=W.QCheckBox('Follow GT trajectory')
        self.follow_gt.setToolTip('Enable GT pose/path and world TF. RViz follows the LiDAR in the GT world frame and shows only the current scan. An open player-owned RViz is restarted when this mode changes.')
        self.follow_gt.toggled.connect(self.change_follow_gt);row.addWidget(self.follow_gt)
        self.software_rviz=W.QCheckBox('Software RViz');self.software_rviz.setToolTip('Use Mesa software OpenGL for RViz; useful when a container has no working GPU graphics forwarding.')
        self.software_rviz.setChecked(os.environ.get('CONPR_RVIZ_SOFTWARE','1' if Path('/.dockerenv').exists() else '0')=='1');row.addWidget(self.software_rviz)
        self.rviz=W.QPushButton('Open RViz');self.rviz.clicked.connect(self.open_rviz);row.addWidget(self.rviz);layout.addLayout(row)
        row=W.QHBoxLayout();self.skip=W.QCheckBox('Skip marked intervals');self.skip.toggled.connect(self.change_skip);row.addWidget(self.skip)
        btn=W.QPushButton('Load skip CSV…');btn.clicked.connect(self.skip_csv);row.addWidget(btn)
        self.export_button=W.QPushButton('Export ROS 1 bag…' if ROS1 else 'Export MCAP…');self.export_button.clicked.connect(self.export_bag);row.addWidget(self.export_button)
        self.cancel_button=W.QPushButton('Cancel export');self.cancel_button.clicked.connect(self.export_cancel.set);row.addWidget(self.cancel_button);layout.addLayout(row)
        self.logs=W.QPlainTextEdit();self.logs.setReadOnly(True);self.logs.setMaximumBlockCount(300);self.logs.setMaximumHeight(135);layout.addWidget(self.logs)
        self.timer=C.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)
        self.scan()
        if args.dataset:
            target=str(Path(args.dataset).resolve());i=self.sessions.findData(target)
            if i>=0:self.sessions.setCurrentIndex(i);C.QTimer.singleShot(0,self.load)
    def log(self,text):self.logs.appendPlainText(text)
    def failed(self,text):
        if self.closing:return
        self.load_button.setEnabled(True);self.scan_button.setEnabled(True);self.log('ERROR: '+text);W.QMessageBox.warning(self,'ConPR++',text)
    def browse(self):
        path=W.QFileDialog.getExistingDirectory(self,'Select ConPRplusplus or sequence',self.root.text())
        if path:self.root.setText(path);self.scan()
    def scan(self):
        try:
            paths=discover(self.root.text());self.sessions.clear()
            for p in paths:self.sessions.addItem(f'{p.parent.name} / {p.name}',str(p))
            self.log(f'Found {len(paths)} sequences')
        except Exception as exc:self.failed(str(exc))
    def dispose(self):
        if self.controller:self.controller.close();self.controller=None
        if self.executor:self.executor.shutdown(timeout_sec=5)
        if self.spin_thread:self.spin_thread.join(timeout=5)
        if self.node:self.node.destroy_node()
        self.executor=None;self.spin_thread=None;self.node=None
    def load(self):
        path=self.sessions.currentData()
        if not path:return
        self.dispose();self.load_button.setEnabled(False);self.scan_button.setEnabled(False);self.log('Loading '+path)
        def work():
            try:self.signals.loaded.emit(Dataset(path))
            except Exception as exc:self.signals.failed.emit(str(exc))
        self.load_thread=threading.Thread(target=work,daemon=True);self.load_thread.start()
    def loaded(self,ds):
        if self.closing:return
        try:
            self.ds=ds;options=Options(**{n:ch.isChecked() for n,ch in self.checks.items()},namespace=self.args.namespace,reliable=not self.args.best_effort)
            factory=MessageFactory(ds,options);self.node=PlayerNode(factory)
            self.latest_image=None;self.latest_points=None;self.last_image=None;self.cloud.points=None;self.cloud.update();self.image.clear()
            def emit(e,data):
                self.node.emit(e,data)
                if e.sensor=='cam':self.latest_image=(e.stamp,data)
                elif e.sensor=='livox':self.latest_points=data
            def reset(t):
                self.node.reset(t);self.latest_image=None;self.latest_points=None
            self.controller=Playback(ds,emit,reset,rate=self.rate.value(),start_s=self.start.value(),end_s=self.end.value() or None,skip_invalid=self.args.skip_invalid,on_finish=self.node.finish)
            self.controller.enabled={'cam':options.camera,'livox':options.lidar,'imu':options.imu,'gt':options.gt};self.controller.loop=self.loop.isChecked();self.node.controller=self.controller
            self.node.set_parameters([Parameter('rate',value=float(self.rate.value())),Parameter('loop',value=self.loop.isChecked())])
            skip_file=getattr(self.args,'skip_intervals',None)
            if skip_file:
                self.controller.skip_intervals=load_intervals(skip_file,ds.duration);self.skip.setChecked(True)
            self.controller.skip_marked=self.skip.isChecked()
            self.executor=SingleThreadedExecutor();self.executor.add_node(self.node);self.spin_thread=threading.Thread(target=spin_executor,args=(self.executor,),daemon=True);self.spin_thread.start();self.node.reset(self.controller.position)
            self.start.setMaximum(ds.duration);self.end.setMaximum(ds.duration)
            self.details.setText(f'{ds.site} / {ds.sequence}   ·   {ds.duration:.3f} s   ·   {ds.counts["livox"]:,} LiDAR / {ds.counts["cam"]:,} RGB / {ds.counts["imu"]:,} IMU   ·   {DISPLAY_NAME}')
            self.log('Loaded. GT world TF is optional; keep off when feeding SLAM.');self.load_button.setEnabled(True);self.scan_button.setEnabled(True)
        except Exception as exc:self.dispose();self.failed(str(exc))
    def control(self,action):
        if self.controller:
            try:getattr(self.controller,action)()
            except Exception as exc:self.failed(str(exc))
    def change_rate(self,value):
        if self.controller:self.node.set_parameters([Parameter('rate',value=float(value))])
    def change_loop(self,value):
        if self.controller:
            self.node.set_parameters([Parameter('loop',value=bool(value))])
    def toggle(self,name,value):
        if not self.controller:return
        try:
            with self.controller.cv:
                if name=='custom' and value:self.node.factory.require_custom()
                if name=='custom':self.node.factory.options.custom=value
                else:self.node.set_parameters([Parameter(name,value=bool(value))])
        except Exception as exc:self.failed(str(exc))
    def change_follow_gt(self,value):
        if value:
            self.follow_previous={n:self.checks[n].isChecked() for n in ('gt','gt_tf')}
            for n in ('gt','gt_tf'):
                self.checks[n].setChecked(True);self.checks[n].setEnabled(False)
        else:
            for n in ('gt','gt_tf'):
                self.checks[n].setEnabled(True)
                if self.follow_previous is not None:self.checks[n].setChecked(self.follow_previous[n])
            self.follow_previous=None
        self.log('Follow GT trajectory '+('enabled: press Play to publish GT transforms.' if value else 'disabled.'))
        if self.rviz_process and self.rviz_process.poll() is None and not self.rviz_restarting:
            self.rviz_restarting=True;self.rviz_process.terminate()
            self.log('Restarting player-owned RViz to apply the view settings.')
            self.wait_rviz_restart(time.monotonic()+5)
    def wait_rviz_restart(self,deadline):
        if self.closing:return
        if self.rviz_process.poll() is None:
            if time.monotonic()<deadline:
                C.QTimer.singleShot(100,lambda:self.wait_rviz_restart(deadline));return
            self.rviz_restarting=False
            self.log('RViz has not exited. Close its window, then click Open RViz.');return
        self.rviz_restarting=False;self.open_rviz()
    def seek_slider(self):
        if self.controller:
            lo=(self.ds.events[self.controller.lo].stamp-self.ds.start)/1e9;hi=(self.ds.events[self.controller.hi-1].stamp-self.ds.start)/1e9
            try:self.controller.seek(lo+(hi-lo)*self.slider.value()/10000)
            except Exception as exc:self.failed(str(exc))
    def skip_csv(self):
        if not self.controller:return
        path,_=W.QFileDialog.getOpenFileName(self,'Skip intervals: start_s,end_s','','CSV (*.csv)')
        if path:
            try:
                intervals=load_intervals(path,self.ds.duration)
                with self.controller.cv:self.controller.skip_intervals=intervals
                self.log(f'{len(intervals)} marked intervals loaded; original timestamps retained.')
            except Exception as exc:self.failed(str(exc))
    def change_skip(self,value):
        if self.controller:
            with self.controller.cv:self.controller.skip_marked=value
    def refresh(self):
        if not self.controller:return
        s=self.controller.snapshot();self.status.setText(f'{s["state"]}   {s["position_s"]:.3f} / {s["duration_s"]:.3f} s   |   {s["range_progress"]*100:.1f}%   |   sent {sum(s["published_events"].values()):,}   |   late >100ms {sum(s["late_events"].values())}   |   loops {s["loops"]}')
        # Reflect service/parameter control without re-emitting GUI change signals.
        with C.QSignalBlocker(self.rate):self.rate.setValue(s['rate'])
        with C.QSignalBlocker(self.loop):self.loop.setChecked(self.controller.loop)
        for name,ch in self.checks.items():
            with C.QSignalBlocker(ch):ch.setChecked(getattr(self.node.factory.options,name))
        if self.follow_gt.isChecked() and not (self.checks['gt'].isChecked() and self.checks['gt_tf'].isChecked()):
            # External ROS parameter control takes precedence; do not turn GT back on.
            self.follow_previous=None;self.follow_gt.setChecked(False)
        if not self.slider.isSliderDown():
            lo=(self.ds.events[self.controller.lo].stamp-self.ds.start)/1e9;hi=(self.ds.events[self.controller.hi-1].stamp-self.ds.start)/1e9
            self.slider.setValue(round(10000*(s['position_s']-lo)/max(hi-lo,1e-9)))
        latest=self.latest_image
        if latest is not None and latest[0]!=self.last_image:
            t,data=latest;im=G.QImage(data.data,data.shape[1],data.shape[0],data.shape[1]*3,G.QImage.Format_RGB888).copy();self.image.setPixmap(G.QPixmap.fromImage(im).scaled(self.image.size(),C.Qt.KeepAspectRatio,C.Qt.SmoothTransformation));self.last_image=t
        if latest is None and self.last_image is not None:self.image.clear();self.last_image=None
        self.cloud.points=self.latest_points;self.cloud.update()
        if s['state']=='ERROR' and s['errors']:
            text=s['errors'][-1]
            if getattr(self,'last_error',None)!=text:self.log('ERROR: '+text);self.last_error=text
    def open_rviz(self):
        if self.rviz_restarting:return
        if self.rviz_process and self.rviz_process.poll() is None:return
        from .resources import config_path
        config=config_path('player_ros1.rviz' if ROS1 else 'player.rviz')
        settings=yaml.safe_load(config.read_text());manager=settings['Visualization Manager']
        follow=self.follow_gt.isChecked()
        manager['Global Options']['Fixed Frame']='conpr_gt_world' if follow else 'conpr_lidar'
        manager['Views']['Current']['Target Frame']='conpr_lidar'
        for display in manager['Displays']:
            kind=display['Class'].split('/')[-1]
            if kind=='PointCloud2':display['Decay Time']=0
            if kind=='Path':display['Enabled']=follow;display['Name']='GT trajectory'
            topic_key='Image Topic' if ROS1 and kind=='Image' else 'Topic'
            topic=display.get(topic_key)
            if topic:
                original=topic if isinstance(topic,str) else topic['Value']
                name='/'+('/'.join(x.strip('/') for x in (self.args.namespace,original) if x.strip('/')))
                if isinstance(topic,str):display[topic_key]=name
                else:
                    topic['Value']=name
                    if kind in ('PointCloud2','Image') and self.args.best_effort:topic['Reliability Policy']='Best Effort'
        config=Path(self.rviz_configs.name)/'player.rviz';config.write_text(yaml.safe_dump(settings,sort_keys=False))
        env=os.environ.copy()
        if self.software_rviz.isChecked():env.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
        else:
            env.pop('LIBGL_ALWAYS_SOFTWARE',None);env.pop('__GLX_VENDOR_LIBRARY_NAME',None)
        try:
            self.rviz_process=subprocess.Popen(['rviz','-d',str(config)] if ROS1 else ['rviz2','-d',str(config),'--ros-args','-p','use_sim_time:=true'],env=env)
            self.log('Starting '+('RViz' if ROS1 else 'RViz2')+(' (software OpenGL)' if self.software_rviz.isChecked() else ' (system OpenGL)'))
        except Exception as exc:self.failed(str(exc))
    def export_bag(self):
        if not self.ds or (self.export_thread and self.export_thread.is_alive()):return
        dest=W.QFileDialog.getExistingDirectory(self,'Select parent for NEW bag')
        if not dest:return
        path=Path(dest)/f'{self.ds.site}_{self.ds.sequence}_{time.strftime("%Y%m%d_%H%M%S")}'
        if ROS1:path=path.with_suffix('.bag')
        from .bag import export
        options=copy.copy(self.node.factory.options);ds=self.ds;start=self.start.value();end=self.end.value() or None
        intervals=list(self.controller.skip_intervals) if self.skip.isChecked() else []
        self.export_cancel.clear();self.export_button.setEnabled(False);self.controller.pause();self.log('Exporting '+str(path))
        def work():
            try:
                result=export(ds,path,options,start,end,lambda i,n,c:self.signals.export_update.emit(f'EXPORT {i}/{n} ({100*i/n:.1f}%)'),self.export_cancel,intervals)
                self.signals.export_done.emit('Export complete: '+str(path)+'\n'+json.dumps(result['counts']))
            except Exception as exc:self.signals.export_done.emit('Export stopped: '+str(exc))
        self.export_thread=threading.Thread(target=work,daemon=True);self.export_thread.start()
    def export_done(self,text):self.export_button.setEnabled(True);self.log(text)
    def closeEvent(self,event):
        self.closing=True
        self.export_cancel.set()
        if self.export_thread:self.export_thread.join(timeout=15)
        if hasattr(self,'load_thread'):self.load_thread.join(timeout=15)
        self.dispose();event.accept()

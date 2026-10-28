import argparse,json,time,threading,signal
from pathlib import Path
from .dataset import Dataset,discover,load_intervals

def main():
    p=argparse.ArgumentParser(description='ConPR++ Noetic / Humble / Jazzy file player')
    p.add_argument('command',choices=['gui','play','validate','export'])
    p.add_argument('--root');p.add_argument('--dataset');p.add_argument('--output');p.add_argument('--full',action='store_true')
    p.add_argument('--start',type=float,default=0.);p.add_argument('--end',type=float);p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--gt',action='store_true');p.add_argument('--gt-tf',action='store_true');p.add_argument('--custom',action='store_true')
    p.add_argument('--no-camera',action='store_true');p.add_argument('--no-lidar',action='store_true');p.add_argument('--no-imu',action='store_true')
    p.add_argument('--loop',action='store_true');p.add_argument('--paused',action='store_true');p.add_argument('--skip-invalid',action='store_true')
    p.add_argument('--skip-intervals');p.add_argument('--namespace',default='');p.add_argument('--best-effort',action='store_true')
    p.add_argument('--storage',choices=['bag','mcap','sqlite3'],help='Export storage; default ROS 1 bag / ROS 2 MCAP')
    p.add_argument('--startup-delay',type=float,default=2.);p.add_argument('--summary')
    from . import backend
    args=p.parse_args(backend.cli_args())
    if args.end==0:args.end=None
    if args.command=='validate':
        paths=discover(args.root) if args.root else [Path(args.dataset)] if args.dataset else []
        if not paths:p.error('Specify --root or --dataset')
        rows=[]
        for path in paths:
            print('VALIDATING',path,flush=True)
            ds=Dataset(path);result=ds.validate(args.full,lambda i:print('DECODED',i,flush=True));rows.append(result);print(json.dumps(result),flush=True)
        if args.output:Path(args.output).write_text(json.dumps(rows,indent=2)+'\n')
        return
    from .ros import Options,MessageFactory,PlayerNode,spin_executor
    from .controller import Playback
    if args.gt_tf and not args.gt:p.error('--gt-tf requires --gt')
    options=Options(camera=not args.no_camera,lidar=not args.no_lidar,imu=not args.no_imu,gt=args.gt,gt_tf=args.gt_tf,custom=args.custom,namespace=args.namespace,reliable=not args.best_effort)
    if args.command=='export':
        if not args.dataset or not args.output:p.error('export needs --dataset and --output')
        from .bag import export
        ds=Dataset(args.dataset);intervals=load_intervals(args.skip_intervals,ds.duration) if args.skip_intervals else []
        print(json.dumps(export(ds,args.output,options,args.start,args.end,lambda i,n,c:print('EXPORT',i,'/',n,c,flush=True),skip_intervals=intervals,storage=args.storage),indent=2));return
    backend.init()
    if args.command=='gui':
        from PyQt5.QtWidgets import QApplication
        from .gui import Window
        app=QApplication([]);window=Window(args);window.show()
        signal.signal(signal.SIGINT,lambda *_:window.close())
        signal.signal(signal.SIGTERM,lambda *_:window.close())
        try:app.exec_()
        finally:
            window.dispose()
            backend.shutdown()
        return
    if not args.dataset:p.error('play needs --dataset')
    from .backend import SingleThreadedExecutor
    ds=Dataset(args.dataset);node=PlayerNode(MessageFactory(ds,options));ctl=Playback(ds,node.emit,node.reset,args.rate,args.start,args.end,args.skip_invalid,on_finish=node.finish)
    ctl.enabled={'cam':options.camera,'livox':options.lidar,'imu':options.imu,'gt':options.gt};ctl.loop=args.loop;node.controller=ctl
    from .backend import Parameter
    node.set_parameters([Parameter('rate',value=float(args.rate)),Parameter('loop',value=args.loop)])
    if args.skip_intervals:ctl.skip_intervals=load_intervals(args.skip_intervals,ds.duration);ctl.skip_marked=True
    executor=SingleThreadedExecutor();executor.add_node(node);worker=threading.Thread(target=spin_executor,args=(executor,),daemon=True);worker.start()
    try:
        node.reset(ctl.position);time.sleep(args.startup_delay)
        if not args.paused:ctl.play()
        while backend.ok():
            s=ctl.snapshot();print(json.dumps(s),flush=True)
            if s['state'] in ('ERROR','FINISHED'):break
            time.sleep(1)
    except KeyboardInterrupt:pass
    finally:
        ctl.close();executor.shutdown(timeout_sec=5);worker.join(timeout=5)
        result=ctl.snapshot();result['ros_messages']=node.message_counts
        if args.summary:Path(args.summary).write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        backend.shutdown()
    if result['state']=='ERROR':raise SystemExit(1)

if __name__=='__main__':main()

"""Shared controller commands; transport implementations supply publishers/services."""
import json
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Path
from .messages import header,TransformStamped,TFMessage,fill_vector,fill_quaternion

class NodeControl:
    def _trigger(self,name):
        def callback(req,res):
            try:
                if self.controller is None:raise RuntimeError('No dataset loaded')
                getattr(self.controller,name)();res.success=True;res.message=self.controller.state
            except Exception as exc:res.success=False;res.message=str(exc)
            return res
        return callback
    def _pause(self,req,res):return self._trigger('pause' if req.data else 'play')(req,res)
    def _apply_parameters(self,params):
        try:
            if self.controller is None:return self.parameter_result(successful=False,reason='Not ready')
            for p in params:
                if p.name=='rate' and not 0<float(p.value)<=100:raise ValueError('rate must be in (0,100]')
                if p.name=='seek_s':
                    t=self.factory.ds.start+round(float(p.value)*1e9)
                    if not self.factory.ds.events[self.controller.lo].stamp<=t<=self.factory.ds.events[self.controller.hi-1].stamp:raise ValueError('seek outside range')
            with self.controller.cv:
                for p in params:
                    if p.name=='rate':self.controller.set_rate(p.value)
                    elif p.name=='seek_s':self.controller.seek(p.value)
                    elif p.name=='loop':self.controller.loop=bool(p.value)
                    elif p.name in ('camera','lidar','imu','gt'):
                        setattr(self.factory.options,p.name,bool(p.value));self.controller.set_enabled({'camera':'cam','lidar':'livox'}.get(p.name,p.name),p.value)
                        if p.name=='gt' and not p.value:self.publish_messages([('path',self.factory.reset(self.controller.position)[-1][1])])
                    elif p.name=='gt_tf':self.factory.options.gt_tf=bool(p.value)
            return self.parameter_result(successful=True)
        except Exception as exc:return self.parameter_result(successful=False,reason=str(exc))
    def publish_messages(self,items):
        for key,msg in items:
            if key not in self.pubs:
                if key=='custom':self.pubs[key]=self.create_publisher(self.factory.CustomMsg,self.factory.topic(key),self.qos)
                else:continue
            self.pubs[key].publish(msg);self.message_counts[key]=self.message_counts.get(key,0)+1
    def _preview_gt_tf(self,t):
        """Buffer original GT TF up to 0.5 s ahead for offline visualization.

        Jazzy's point-cloud display can attempt TF lookup immediately on arrival.
        The original cloud stamp often lies just AFTER the latest GT stamp; without
        a later GT pose in the buffer, the cloud is discarded. Only live optional
        GT TF delivery is prefetched. Sensor/pose/path stamps and bag export remain
        unchanged. No extrapolated or interpolated poses are generated here.
        """
        enabled=self.factory.options.gt and self.factory.options.gt_tf
        if not enabled:
            self._gt_tf_next=None;return False
        ds=self.factory.ds
        i=getattr(self,'_gt_tf_next',None)
        if i is None:
            i=max(0,int(np.searchsorted(ds.gt_ts,t,side='right'))-1)
        end=int(np.searchsorted(ds.gt_ts,t+500_000_000,side='right'))
        transforms=[]
        for j in range(i,end):
            pose=ds.gt_values[j]
            tf=TransformStamped(header=header(int(ds.gt_ts[j]),self.factory.frames['gt_world']),child_frame_id=self.factory.frames['imu'])
            fill_vector(tf.transform.translation,pose[:3]);fill_quaternion(tf.transform.rotation,pose[3:7]);transforms.append(tf)
        self._gt_tf_next=end
        if transforms:self.publish_messages([('tf',TFMessage(transforms=transforms))])
        return True
    def emit(self,event,data):
        prefetched=self._preview_gt_tf(event.stamp)
        items=self.factory.messages(event,data)
        if prefetched:items=[(key,msg) for key,msg in items if key!='tf']
        self.publish_messages(items)
    def reset(self,t):
        self._gt_tf_next=None
        self.publish_messages(self.factory.reset(t))
        self._preview_gt_tf(t)
    def finish(self,t):
        if self.factory.poses:self.publish_messages([('path',Path(header=header(t,self.factory.frames['gt_world']),poses=list(self.factory.poses)))])
    def _status(self):
        if self.controller:
            s=self.controller.snapshot();s['ros_messages']=dict(self.message_counts);self.status_pub.publish(String(data=json.dumps(s)))

"""ROS-independent playback state machine with bounded asynchronous prefetch."""
import threading,time
from concurrent.futures import ThreadPoolExecutor
from collections import Counter
import numpy as np

class Playback:
    def __init__(self,dataset,emit,reset=lambda t:None,rate=1.,start_s=0.,end_s=None,skip_invalid=False,prefetch=16,on_finish=lambda t:None):
        self.ds=dataset;self.emit=emit;self.reset=reset;self.on_finish=on_finish
        self.lo,self.hi=dataset.bounds(start_s,end_s);self.i=self.lo
        if self.lo>=self.hi:raise ValueError('Empty playback range')
        self.cv=threading.Condition(threading.RLock());self.state='READY';self.rate=float(rate)
        if not 0<self.rate<=100:raise ValueError('Rate must be in (0,100]')
        self.loop=False;self.closed=False;self.epoch=0;self.step_end=None
        self.position=dataset.events[self.lo].stamp;self.base_stamp=self.position;self.base_wall=time.monotonic()
        self.enabled={'cam':True,'livox':True,'imu':True,'gt':False}
        self.skip_invalid=skip_invalid;self.skip_intervals=[];self.skip_marked=False
        self.counts=Counter();self.skipped=Counter();self.late=Counter();self.errors=[];self.max_late_s=0.;self.loops=0
        self.pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix='conpr-prefetch');self.cache={};self.prefetch=prefetch
        self.thread=threading.Thread(target=self._run,daemon=True,name='conpr-playback');self.thread.start()

    def _anchor(self):self.base_wall=time.monotonic();self.base_stamp=self.position
    def _clear(self):
        self.epoch+=1
        for f in self.cache.values():f.cancel()
        self.cache.clear();self.step_end=None
    def _seek_locked(self,index):
        self._clear();self.i=max(self.lo,min(index,self.hi-1));self.position=self.ds.events[self.i].stamp
        self._anchor();self.reset(self.position)

    def play(self):
        with self.cv:
            if self.state=='ERROR':raise RuntimeError('Playback error; restart after resolving input')
            if self.state=='FINISHED':self._seek_locked(self.lo)
            self._anchor();self.step_end=None;self.state='PLAYING';self.cv.notify_all()
    def pause(self):
        with self.cv:
            if self.state=='PLAYING':self.state='PAUSED';self.step_end=None
            self.cv.notify_all()
    def stop(self):
        with self.cv:self.state='STOPPED';self._seek_locked(self.lo);self.cv.notify_all()
    def restart(self):
        with self.cv:self._seek_locked(self.lo);self.state='PLAYING';self.cv.notify_all()
    def seek(self,seconds):
        with self.cv:
            t=self.ds.start+round(float(seconds)*1e9)
            if t<self.ds.events[self.lo].stamp or t>self.ds.events[self.hi-1].stamp:raise ValueError('Seek outside selected range')
            state=self.state;self._seek_locked(int(np.searchsorted(self.ds.times,t,'left')))
            self.state='PLAYING' if state=='PLAYING' else 'PAUSED';self.cv.notify_all()
    def set_rate(self,rate):
        rate=float(rate)
        if not 0<rate<=100:raise ValueError('Rate must be in (0,100]')
        with self.cv:self.rate=rate;self._anchor();self.cv.notify_all()
    def set_enabled(self,sensor,value):
        with self.cv:
            if sensor not in self.enabled:raise ValueError(sensor)
            self.enabled[sensor]=bool(value);self.cv.notify_all()
    def step(self):
        with self.cv:
            if self.i>=self.hi:self._seek_locked(self.lo)
            future=self.ds.lidar_indices[self.ds.lidar_indices>=self.i]
            self.step_end=min(int(future[0])+1,self.hi) if len(future) else self.hi
            self._anchor();self.state='PLAYING';self.cv.notify_all()
    def snapshot(self):
        with self.cv:
            return dict(state=self.state,position_s=(self.position-self.ds.start)/1e9,duration_s=self.ds.duration,
                        event_index=self.i,range_events=self.hi-self.lo,range_progress=(self.i-self.lo)/(self.hi-self.lo),rate=self.rate,
                        published_events=dict(self.counts),skipped_events=dict(self.skipped),late_events=dict(self.late),max_late_s=self.max_late_s,
                        errors=self.errors[-10:],loops=self.loops,epoch=self.epoch)
    def close(self):
        with self.cv:self.closed=True;self.cv.notify_all()
        self.thread.join(timeout=15)
        for future in self.cache.values():future.cancel()
        self.pool.shutdown(wait=True)
        if self.thread.is_alive():raise RuntimeError('Playback thread did not stop')
    def _prefetch(self):
        for j in range(self.i,min(self.i+self.prefetch,self.hi)):
            if j not in self.cache and self.enabled[self.ds.events[j].sensor]:self.cache[j]=self.pool.submit(self.ds.decode,self.ds.events[j])
    def _run(self):
        while True:
            with self.cv:
                while self.state!='PLAYING' and not self.closed:self.cv.wait()
                if self.closed:return
                if self.i>=self.hi:
                    try:self.on_finish(self.position)
                    except Exception as exc:self.errors.append(str(exc));self.state='ERROR';continue
                    if self.loop:self.loops+=1;self._seek_locked(self.lo)
                    else:self.state='FINISHED';continue
                event=self.ds.events[self.i]
                rel=(event.stamp-self.ds.start)/1e9
                interval=next(((a,b) for a,b in self.skip_intervals if a<=rel<b),None) if self.skip_marked else None
                if interval:
                    stop=min(self.hi,int(np.searchsorted(self.ds.times,self.ds.start+round(interval[1]*1e9),'left')))
                    self.skipped['marked_interval']+=stop-self.i
                    self._clear();self.i=stop
                    if stop<self.hi:self.position=self.ds.events[stop].stamp;self._anchor();self.reset(self.position)
                    continue
                if not self.enabled[event.sensor]:
                    self.skipped['disabled_'+event.sensor]+=1;self.i+=1
                    old=self.cache.pop(self.i-1,None)
                    if old:old.cancel()
                    if self.step_end is not None and self.i>=self.step_end:self.state='PAUSED';self.step_end=None
                    continue
                self._prefetch();i,epoch=self.i,self.epoch;f=self.cache[i]
            try:decoded=f.result()
            except Exception as exc:
                with self.cv:
                    if epoch!=self.epoch:continue
                    self.errors.append(f'{event.sensor}@{event.stamp}: {exc}')
                    if self.skip_invalid:self.skipped['invalid_'+event.sensor]+=1;self.cache.pop(i,None);self.i+=1
                    else:self.state='ERROR'
                continue
            with self.cv:
                if self.closed:return
                if epoch!=self.epoch or self.state!='PLAYING' or self.i!=i:continue
                target=self.base_wall+(event.stamp-self.base_stamp)/1e9/self.rate
                remaining=target-time.monotonic()
                if self.step_end is None and remaining>0:
                    self.cv.wait(min(remaining,.05));continue
                delay=max(0.,time.monotonic()-target) if self.step_end is None else 0.
                try:self.emit(event,decoded)
                except Exception as exc:
                    self.errors.append(str(exc));self.state='ERROR';continue
                self.counts[event.sensor]+=1
                if delay>.1:self.late[event.sensor]+=1
                self.max_late_s=max(self.max_late_s,delay);self.position=event.stamp;self.i+=1;self.cache.pop(i,None)
                if self.step_end is not None and self.i>=self.step_end:self.state='PAUSED';self.step_end=None

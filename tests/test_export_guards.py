"""Export format compatibility and failure cases on the actual native writer."""
import json,os,threading
from pathlib import Path
import pytest
from conpr_player.environment import ROS1,ROS_DISTRO
from conpr_player.dataset import Dataset
from conpr_player.bag import export
from conpr_player.messages import Options

@pytest.fixture
def ds():
    root=Path(os.environ.get('CONPR_DATA_ROOT',str(Path(__file__).resolve().parents[3]/'ConPRplusplus')))
    return Dataset(root/'Hall/260905')

def destination(tmp_path):return tmp_path/('test.bag' if ROS1 else 'test')
def sidecar(path):return Path(str(path)+'.conpr_export.json') if ROS1 else path/'conpr_export.json'

def test_cancel_retains_incomplete_marker(ds,tmp_path):
    event=threading.Event();event.set();path=destination(tmp_path)
    with pytest.raises(InterruptedError):export(ds,path,end_s=.1,cancel=event)
    result=json.loads(sidecar(path).read_text());assert result['complete'] is False and result['error']
    with pytest.raises(FileExistsError):export(ds,path,end_s=.1)

def test_decode_error_is_not_silently_skipped(ds,tmp_path,monkeypatch):
    def fail(event):raise ValueError('intentional decode failure')
    monkeypatch.setattr(ds,'decode',fail);path=destination(tmp_path)
    with pytest.raises(ValueError,match='intentional'):export(ds,path,end_s=.1)
    result=json.loads(sidecar(path).read_text());assert not result['complete']

def test_sensor_filter_and_range(ds,tmp_path):
    path=destination(tmp_path);o=Options(camera=False,lidar=False,gt=False)
    result=export(ds,path,o,start_s=.1,end_s=.2)
    lo,hi=ds.bounds(.1,.2)
    assert result['counts']['imu']==sum(e.sensor=='imu' for e in ds.events[lo:hi])
    assert not ({'camera','points','pose','custom'} & result['counts'].keys())
    assert result['ros_distro']==ROS_DISTRO

def test_metadata_qos_and_sqlite(ds,tmp_path):
    if ROS1:pytest.skip('ROS 1 export tested by native .bag probe')
    from conpr_player.bag import ros2_topic_metadata
    import yaml
    for key in ('points','static'):
        meta=ros2_topic_metadata(key,'/test',1,Options(reliable=False))
        if ROS_DISTRO=='humble':
            assert isinstance(meta.offered_qos_profiles,str)
            q=yaml.safe_load(meta.offered_qos_profiles)[0]
            assert q['reliability']==(1 if key=='static' else 2)
            assert q['durability']==(1 if key=='static' else 2)
        else:assert len(meta.offered_qos_profiles)==1
    result=export(ds,tmp_path/'sqlite',end_s=.1,storage='sqlite3')
    assert result['complete'] and list((tmp_path/'sqlite').glob('*.db3'))

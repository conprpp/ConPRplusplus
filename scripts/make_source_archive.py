"""Create a portable source release, excluding data, local runtimes and build products."""
import argparse,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args()
r=Path(__file__).resolve().parents[1];dest=Path(a.output).resolve()
if dest.exists():raise FileExistsError(dest)
dest.parent.mkdir(parents=True,exist_ok=True)
entries=['README.md','LICENSE','THIRD_PARTY.md','.gitignore','.github','build.sh','run.sh','conpr_player','ros1','ros2','config','scripts','tests']
entries+=['.dockerignore']+[str(Path('docker')/name) for name in ('Dockerfile','entrypoint.sh','create-user.sh','shell-setup.sh','compose.yaml','prepare-host.sh','README.md','TESTING.md')]
with tarfile.open(dest,'w:gz') as tar:
    for entry in entries:
        base=r/entry
        files=sorted(base.rglob('*')) if base.is_dir() else [base]
        for f in files:
            if not f.is_file() or any(x in ('__pycache__','.pytest_cache') or x.endswith('.egg-info') for x in f.parts) or f.suffix=='.pyc':continue
            if f.relative_to(r)==Path('config/player.launch.py'):continue
            if f.name in ('runtime_probe.py','services_bag_probe.py','rviz_probe.py'):continue  # historical workspace-only tests
            tar.add(f,arcname=str(Path('conpr_file_player')/f.relative_to(r)),recursive=False)
print(dest)

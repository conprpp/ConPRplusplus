"""Use a supplied dataset, or generate a small public-format fixture locally."""
import os
import subprocess
import sys
from pathlib import Path
import pytest


@pytest.fixture(scope='session')
def data_root(tmp_path_factory):
    supplied = os.environ.get('CONPR_DATA_ROOT')
    if supplied:
        root = Path(supplied).expanduser().resolve()
        if not root.is_dir():
            pytest.fail('CONPR_DATA_ROOT does not exist: ' + str(root))
        return root
    root = tmp_path_factory.mktemp('conpr_fixture') / 'ConPRplusplus'
    subprocess.run([sys.executable, str(Path(__file__).with_name('make_fixture.py')), str(root)], check=True)
    return root

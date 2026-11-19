# Copyright (c) 2026 camera_aravis2 contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Fraunhofer IOSB nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('final_status', [0, 1])
def test_retry_only_unavailable(tmp_path, final_status):
    # Launch a substitute driver through normal ROS package lookup so the test needs
    # neither a connected camera nor a free GigE discovery port.
    package = 'camera_aravis2'
    marker = tmp_path / 'share' / 'ament_index' / 'resource_index' / 'packages' / package
    marker.parent.mkdir(parents=True)
    marker.touch()
    attempts = tmp_path / 'attempts'
    driver = tmp_path / 'lib' / package / 'camera_driver_gv'
    driver.parent.mkdir(parents=True)
    driver.write_text(f"""#!{sys.executable}
from pathlib import Path
log = Path({str(attempts)!r})
with log.open("a") as out:
    out.write("start\\n")
raise SystemExit({os.EX_TEMPFAIL} if len(log.read_text().splitlines()) == 1 else {final_status})
""")
    driver.chmod(0o755)

    launch_dir = Path(__file__).resolve().parents[1] / 'launch'
    launch_file = launch_dir / 'camera_driver_reconnect.launch.py'
    env = dict(os.environ)
    env['AMENT_PREFIX_PATH'] = os.pathsep.join([str(tmp_path), env['AMENT_PREFIX_PATH']])
    env['ROS_LOG_DIR'] = str(tmp_path / 'log')
    result = subprocess.run(
        ['ros2', 'launch', str(launch_file), 'guid:=test-camera'],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20,
    )
    assert result.returncode == 0, result.stdout
    assert attempts.read_text().splitlines() == ['start', 'start'], result.stdout

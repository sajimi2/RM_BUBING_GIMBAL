"""在临时假安装目录验证控件打包和备份，不接触正在运行的VOFA。"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
NAMES = ('vision_replay.qml', 'replay.js', 'recording.js', 'simulation.js', 'dual_recording.js')
if os.name != 'nt':
    raise SystemExit('SKIP: PowerShell installer test requires Windows')

with tempfile.TemporaryDirectory(prefix='vofa-replay-install-test-') as temporary:
    target = Path(temporary).resolve()
    widget = target / 'plugins/widgets/vision_replay'
    widget.mkdir(parents=True)
    unrelated = target / 'plugins/widgets/yaw_sweep/keep.txt'
    unrelated.parent.mkdir()
    unrelated.write_text('existing widget', encoding='utf-8')
    for name in NAMES:
        (widget / name).write_bytes(('old ' + name).encode())
    subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                    str(ROOT / 'tools/vofa/install_replay.ps1'), '-VofaDirectory', str(target)], check=True)
    backups = list((target / 'widget-backups').iterdir())
    assert len(backups) == 1
    for name in NAMES:
        assert (widget / name).read_bytes() == (ROOT / 'tools/vofa/vision_replay' / name).read_bytes()
        assert (backups[0] / name).read_bytes() == ('old ' + name).encode()
    assert unrelated.read_text(encoding='utf-8') == 'existing widget'

print('PASS: all five QML/JS dependencies installed and verified in fake directory / old files backed up / unrelated widget preserved / no real VOFA touched')

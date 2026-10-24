"""检查提取范围、CSV数值与原件保护；不连接串口。"""
from pathlib import Path
import csv
import importlib.util
import json
import tempfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('prepare_vision_replay', ROOT / 'tools/prepare_vision_replay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    source = directory / '实测.csv'
    header = ['Pitch实际', 'Pitch目标', 'Yaw实际', 'Yaw目标'] + ['其他'] * 8
    rows = [[0, 0, 0, value, 0, 1, 0, 0, 0, 0, 0, 0] for value in [0, 2, -7, -7, 1]]
    with source.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)
    original = source.read_bytes()
    report = module.prepare(source, directory / 'output', 2, 5, 20)
    assert source.read_bytes() == original
    assert report['selected_samples'] == 4
    assert report['max_observed_step_deg'] == 9
    text = (directory / 'output/recording.js').read_text(encoding='utf-8')
    data = json.loads(text.split('var recording = ', 1)[1].rstrip(';\n'))
    assert data['yawDeg'] == [2, -7, -7, 1]
    assert len(data['profiles']) == 1
    # 新增的诊断列不改变目标列，也不能被误当作目标角使用。
    diagnostics = directory / '23通道.csv'
    with diagnostics.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header + ['诊断'] * 11)
        writer.writerows([row + list(range(11)) for row in rows])
    diagnostics_original = diagnostics.read_bytes()
    header23, rows23, _ = module.read_recording(diagnostics)
    assert len(header23) == 23 and rows23[0][12:] == list(range(11))
    module.prepare(diagnostics, directory / 'diagnostics', 2, 5, 20)
    text23 = (directory / 'diagnostics/recording.js').read_text(encoding='utf-8')
    data23 = json.loads(text23.split('var recording = ', 1)[1].rstrip(';\n'))
    assert data23['yawDeg'] == data['yawDeg']
    assert diagnostics.read_bytes() == diagnostics_original
    profiles = [{'name': '测试片段', 'firstRow': 3, 'lastRow': 5}]
    module.prepare(source, directory / 'profile', 2, 5, 20, profiles, 'Kp=0.012，前馈0')
    text = (directory / 'profile/recording.js').read_text(encoding='utf-8')
    data = json.loads(text.split('var recording = ', 1)[1].rstrip(';\n'))
    assert data['profiles'][1] == profiles[0]
    assert data['baselineLabel'] == 'Kp=0.012，前馈0'
    try:
        module.prepare(source, directory / 'bad-profile', 2, 5, 20,
                       [{'name': '越界', 'firstRow': 1, 'lastRow': 4}])
        raise AssertionError('Invalid profile accepted')
    except ValueError:
        pass
    for first, last, period in [(0, 5, 20), (2, 6, 20), (3, 3, 20), (2, 5, float('nan'))]:
        try:
            module.prepare(source, directory / 'invalid', first, last, period)
            raise AssertionError('Invalid selection accepted')
        except ValueError:
            pass

print('PASS: 12 and 23 channels / unchanged target column / inclusive range / zero and repeated values preserved / exact jumps / source unchanged / invalid settings')

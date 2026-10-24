"""把VOFA记录中的Yaw目标段准备为控件数据；不连接串口、不改原CSV。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parents[1]


def read_recording(path):
    raw = Path(path).read_bytes()
    try:
        text = raw.decode('utf-8-sig')
    except UnicodeDecodeError:
        text = raw.decode('gb18030')
    records = list(csv.reader(io.StringIO(text)))
    expected = ['Pitch', 'Pitch', 'Yaw', 'Yaw']
    if len(records) < 2:
        raise ValueError('记录没有数据行')
    if len(records[0]) != 12:
        raise ValueError('表头应为12通道')
    for index, name in enumerate(expected):
        if name not in records[0][index]:
            raise ValueError('前四列应为Pitch实际/目标角、Yaw实际/目标角，单位为度')
    rows = []
    for number, record in enumerate(records[1:], 1):
        if len(record) != 12:
            raise ValueError(f'数据行{number}不是12通道')
        values = [float(value) for value in record]
        for value in values:
            if not math.isfinite(value):
                raise ValueError(f'数据行{number}含无效数字')
        rows.append(values)
    return records[0], rows, hashlib.sha256(raw).hexdigest()


def velocity_time_fit(rows):
    """角度差约等于平均角速度乘时间；仅作周期交叉估计，不替代时间戳。"""
    numerator = 0.0
    denominator = 0.0
    count = 0
    for previous, current in zip(rows, rows[1:]):
        speed = (previous[7] + current[7]) * 0.5
        if abs(speed) <= 5.0:
            continue
        if abs(speed) >= 180.0:
            continue
        angle_change = current[2] - previous[2]
        if abs(angle_change) >= 5.0:
            continue
        numerator += angle_change * speed
        denominator += speed * speed
        count += 1
    if denominator == 0:
        return {'estimated_ms': None, 'pairs': count}
    return {'estimated_ms': 1000.0 * numerator / denominator, 'pairs': count}


def prepare(source, output, first, last, sample_ms, profiles=None, baseline_label=''):
    header, rows, source_hash = read_recording(source)
    if not 1 <= first < last <= len(rows):
        raise ValueError('开始/结束数据行超出记录范围')
    if not math.isfinite(sample_ms):
        raise ValueError('采样周期必须是有效数字')
    if not 5 <= sample_ms <= 100:
        raise ValueError('采样周期应在5～100 ms之间')

    selected = rows[first - 1:last]
    yaw = [row[3] for row in selected]
    steps = [current - previous for previous, current in zip(yaw, yaw[1:])]
    changed = [abs(step) for step in steps if abs(step) > 0.0001]
    median_step = 0.0
    if changed:
        median_step = statistics.median(changed)
    # 不删跳变、不平滑、不把门控为0的行当成目标角0。
    recording = {
        'source': Path(source).name,
        'sourceSha256': source_hash,
        'firstRow': first,
        'lastRow': last,
        'sampleMs': sample_ms,
        'timebase': 'estimated_no_timestamps',
        'kind': 'recorded_control_setpoint_not_raw_CAN',
        'baselineLabel': baseline_label,
        'yawDeg': yaw,
        'profiles': [{'name': '完整基准段', 'firstRow': first, 'lastRow': last}],
    }
    # 片段由本次数据决定，不能沿用上一份记录的固定行号。
    for profile in profiles or []:
        if not profile['name']:
            raise ValueError('片段名称不能为空')
        if profile['firstRow'] < first:
            raise ValueError('片段开始行超出已提取范围')
        if profile['lastRow'] > last:
            raise ValueError('片段结束行超出已提取范围')
        if profile['firstRow'] >= profile['lastRow']:
            raise ValueError('片段开始行必须小于结束行')
        recording['profiles'].append(dict(profile))

    resampled = []
    elapsed_ms = 0.0
    while elapsed_ms < len(yaw) * sample_ms:
        sample = min(int(elapsed_ms / sample_ms), len(yaw) - 1)
        resampled.append(yaw[sample])
        elapsed_ms += 42.0
    resampled_changes = [abs(b - a) for a, b in zip(resampled, resampled[1:]) if abs(b - a) > 0.0001]

    # 跟踪误差直接由目标与实际角相减；门控字段为0也不能把误差当0。
    online = [row for row in selected if row[5] == 1]
    errors = [(row[3] - row[2] + 180.0) % 360.0 - 180.0 for row in online]
    tracking = {}
    if errors:
        squared_errors = sum(error * error for error in errors)
        tracking['online_samples'] = len(online)
        tracking['rms_error_deg'] = math.sqrt(squared_errors / len(errors))
        tracking['median_abs_error_deg'] = statistics.median(abs(error) for error in errors)
        tracking['max_abs_error_deg'] = max(abs(error) for error in errors)
        tracking['motor_saturated_samples'] = sum(abs(row[4]) >= 30000 for row in online)
        if squared_errors > 0:
            slope = sum(error * row[6] for error, row in zip(errors, online)) / squared_errors
            tracking['target_speed_error_fit_kp_ms'] = slope / 1000.0
            residuals = [row[6] - slope * error for error, row in zip(errors, online)]
            tracking['target_speed_fit_residual_rms_deg_s'] = math.sqrt(statistics.mean(value * value for value in residuals))

    report = {
        'source': str(Path(source).resolve()),
        'source_sha256': source_hash,
        'baseline_label': baseline_label,
        'source_rows': len(rows),
        'selected_first_data_row': first,
        'selected_last_data_row': last,
        'selected_samples': len(selected),
        'estimated_sample_ms': sample_ms,
        'estimated_duration_s': len(selected) * sample_ms / 1000,
        'yaw_target_column': header[3],
        'yaw_target_min_deg': min(yaw),
        'yaw_target_max_deg': max(yaw),
        'max_observed_step_deg': max(changed, default=0.0),
        'median_nonzero_step_deg': median_step,
        'observed_target_changes': len(changed),
        'requested_send_interval_ms': 42,
        'changes_after_42ms_resampling': len(resampled_changes),
        'max_step_after_42ms_resampling_deg': max(resampled_changes, default=0.0),
        'can_offline_samples': sum(row[5] == 0 for row in selected),
        'gate_threshold_zero_samples': sum(row[9] == 0 for row in selected),
        'selected_time_fit': velocity_time_fit(selected),
        'tracking': tracking,
        'blocks': [],
        'limits': [
            '无硬件时间戳，周期由角度/速度交叉估计，不能还原精确CAN收帧时间。',
            '读取控制实际采用的目标角；若录制时开了插值，无法从本CSV恢复原始视觉帧。',
            '只使用指定片段；采集条件由用户说明，CSV没有前馈/插值开关的硬件读回。',
            '发送时约24Hz零阶保持取样；重复帧、原始fire/rev、打弹反冲不在重放中。',
        ],
    }
    for start in range(0, len(rows), 500):
        block = rows[start:start + 500]
        fit = velocity_time_fit(block)
        fit['first_row'] = start + 1
        fit['last_row'] = start + len(block)
        fit['yaw_min_deg'] = min(row[3] for row in block)
        fit['yaw_max_deg'] = max(row[3] for row in block)
        report['blocks'].append(fit)

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    data_js = '.pragma library\n\n// 由实测CSV生成，只包含目标角；原件不变。\nvar recording = '
    data_js += json.dumps(recording, ensure_ascii=False, separators=(',', ':')) + ';\n'
    (output / 'recording.js').write_text(data_js, encoding='utf-8')
    (output / 'recording_analysis.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--first', type=int, required=True, help='数据行，不含表头')
    parser.add_argument('--last', type=int, required=True, help='首尾都包含')
    parser.add_argument('--sample-ms', type=float, default=20)
    parser.add_argument('--profile', action='append', default=[], help='片段名称:开始数据行:结束数据行')
    parser.add_argument('--baseline-label', default='', help='用户说明的采集参数')
    parser.add_argument('--output', type=Path, default=ROOT / 'tools/vofa/vision_replay')
    args = parser.parse_args()
    profiles = []
    for entry in args.profile:
        name, first, last = entry.rsplit(':', 2)
        profiles.append({'name': name, 'firstRow': int(first), 'lastRow': int(last)})
    report = prepare(args.source, args.output, args.first, args.last, args.sample_ms,
                     profiles, args.baseline_label)
    print(json.dumps({key: value for key, value in report.items() if key not in ('blocks', 'limits')},
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

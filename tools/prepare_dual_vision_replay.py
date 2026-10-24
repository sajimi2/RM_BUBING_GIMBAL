"""准备双轴目标和明确的训练/留出片段，不改CSV，不连接设备。"""
import argparse
import json
import math
from pathlib import Path

from prepare_vision_replay import read_recording, velocity_time_fit

ROOT = Path(__file__).resolve().parents[1]


def prepare(source, output, sample_ms, train, holdout, pitch_min, pitch_max, baseline_label):
    header, rows, source_hash = read_recording(source)
    if not math.isfinite(sample_ms) or not 5 <= sample_ms <= 100:
        raise ValueError('记录采样周期应在5～100 ms之间')
    for first, last in (train, holdout):
        if not 1 <= first < last <= len(rows):
            raise ValueError('训练或留出片段越界')
    # 两片段按完整连续区间分开，不能把相邻重复采样随机分到两边。
    if train[1] >= holdout[0]:
        raise ValueError('训练片段必须早于留出片段，且不能重叠')
    if not math.isfinite(pitch_min) or not math.isfinite(pitch_max):
        raise ValueError('Pitch限位必须是有效角度')
    if pitch_min >= pitch_max:
        raise ValueError('Pitch最小限位必须小于最大限位')
    recording = {
        'source': Path(source).name,
        'sourceSha256': source_hash,
        'firstRow': 1,
        'lastRow': len(rows),
        'sampleMs': sample_ms,
        'timebase': 'estimated_no_timestamps',
        'kind': 'recorded_control_setpoint_not_raw_CAN',
        'baselineLabel': baseline_label,
        'pitchLimitsDeg': [pitch_min, pitch_max],
        'yawDeg': [row[3] for row in rows],
        'pitchDeg': [row[1] for row in rows],
        'trainingRows': list(train),
        'holdoutRows': list(holdout),
        'profiles': [
            {'name': '普通跟踪（训练段）', 'firstRow': train[0], 'lastRow': train[1]},
            {'name': '独立留出段（不参与生成）', 'firstRow': holdout[0], 'lastRow': holdout[1]},
            {'name': '完整实录（含切入大跳变）', 'firstRow': 1, 'lastRow': len(rows)},
        ],
    }
    report = {
        'source': str(Path(source).resolve()),
        'source_sha256': source_hash,
        'source_rows': len(rows),
        'pitch_target_column': header[1],
        'yaw_target_column': header[3],
        'estimated_sample_ms': sample_ms,
        'cross_estimated_period': velocity_time_fit(rows),
        'training_data_rows': list(train),
        'holdout_data_rows': list(holdout),
        'baseline_label': baseline_label,
        'limits': ['无收帧时间戳，20ms为估计；不是原始CAN逐帧重放。',
                   '只保存两轴目标；没有学习实际角、电机输出、开火或机械反冲。',
                   'Pitch限位是准备数据时的保守预检范围；固件仍做最终检查。'],
    }
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    text = '.pragma library\n\n// 由实测CSV提取I1/I3，完整保留目标；行号不含表头。\nvar recording = '
    text += json.dumps(recording, ensure_ascii=False, separators=(',', ':')) + ';\n'
    (output / 'dual_recording.js').write_text(text, encoding='utf-8')
    (output / 'dual_recording_analysis.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--sample-ms', type=float, default=20)
    parser.add_argument('--train', type=int, nargs=2, required=True)
    parser.add_argument('--holdout', type=int, nargs=2, required=True)
    parser.add_argument('--pitch-min', type=float, required=True)
    parser.add_argument('--pitch-max', type=float, required=True)
    parser.add_argument('--baseline-label', default='')
    parser.add_argument('--output', type=Path, default=ROOT / 'tools/vofa/vision_replay')
    args = parser.parse_args()
    report = prepare(args.source, args.output, args.sample_ms, args.train, args.holdout,
                     args.pitch_min, args.pitch_max, args.baseline_label)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

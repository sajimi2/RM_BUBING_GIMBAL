"""由静止保持数据拟合 Pitch 重力前馈参数 gain 与 offset_rad。

模型与 gimbal_task.c 一致：u_g = gain * cos(angle + offset_rad)。

采样方法（Ki 开不开都可以，前馈已开也可以用来复标）：
  1. 手动模式把 Pitch 停在若干角度，覆盖尽量大的范围（低头、水平、抬头）。
  2. 等 pitch_control_debug.speed_rad_s 接近 0 后，记录
     pitch_control_debug.actual_angle_rad 与 pitch_control_debug.total_out。
     静止时 D 项约为 0，total_out 就是托住云台所需的输出，单位与方向都和前馈相同。
  3. 静摩擦会让保持输出落在一个区间里：每个角度最好从上方、下方各停一次，都记下来。

输入：CSV 文件（两列 angle_rad,total_out，可有表头），或命令行 "角度,输出" 对。
    python tools/fit_pitch_gravity.py samples.csv
    python tools/fit_pitch_gravity.py -0.3,-1850 0,-2100 0.4,-1900
"""
import csv
import math
import os
import sys

GRAVITY_MAX_OUT = 3000.0  # 与 PITCH_GRAVITY_MAX_OUT 一致
TOTAL_MAX_OUT = 30000.0   # 与 PITCH_VOLT_MAX_OUT 一致


def fit(samples):
    """最小二乘拟合 u = a*cos(t) - b*sin(t)，返回 (gain, offset_rad)。"""
    scc = sss = scs = scu = ssu = 0.0
    for angle, out in samples:
        c, s = math.cos(angle), -math.sin(angle)
        scc += c * c
        sss += s * s
        scs += c * s
        scu += c * out
        ssu += s * out
    det = scc * sss - scs * scs
    if abs(det) < 1e-9:
        raise ValueError('角度太集中，无法区分幅值和偏移；请扩大采样角度范围。')
    a = (scu * sss - ssu * scs) / det
    b = (ssu * scc - scu * scs) / det
    # gain*cos(t+o) = gain*cos(o)*cos(t) - gain*sin(o)*sin(t)
    gain, offset = math.hypot(a, b), math.atan2(b, a)
    # 把 offset 限制在 ±π/2 内、让 gain 带符号：-G*cos(t+o) 与 G*cos(t+o±π) 等价，
    # 前者更直观，也避免 offset 在 ±π 附近因噪声来回跳变
    if offset > math.pi / 2:
        gain, offset = -gain, offset - math.pi
    elif offset < -math.pi / 2:
        gain, offset = -gain, offset + math.pi
    return gain, offset


def load(args):
    samples = []
    if len(args) == 1 and os.path.isfile(args[0]):
        with open(args[0], newline='', encoding='utf-8-sig') as f:
            for row in csv.reader(f):
                if len(row) < 2:
                    continue
                try:
                    samples.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue  # 表头或注释行
    else:
        for item in args:
            angle, out = item.split(',')
            samples.append((float(angle), float(out)))
    return samples


def main(argv=None):
    # 不用 argparse：负角度如 "-0.3,-1850" 会被它当成选项
    args = sys.argv[1:] if argv is None else argv
    if not args or args[0] in ('-h', '--help'):
        print(__doc__)
        return
    samples = load(args)
    if len(samples) < 3:
        sys.exit('至少需要 3 个采样点（2 个未知数，再留 1 个用于检查残差）。')

    gain, offset = fit(samples)
    span = max(a for a, _ in samples) - min(a for a, _ in samples)
    print(f'采样点 {len(samples)} 个，角度范围 {span:.3f} rad（{math.degrees(span):.1f}°）')
    print(f'gain       = {gain:.1f}')
    print(f'offset_rad = {offset:.4f}  （{math.degrees(offset):.1f}°）')
    print()
    print('  angle_rad   total_out   拟合值     残差')
    sq = 0.0
    for angle, out in samples:
        model = gain * math.cos(angle + offset)
        sq += (out - model) ** 2
        print(f'  {angle:9.4f}  {out:10.1f}  {model:9.1f}  {out - model:8.1f}')
    print(f'残差 RMS = {math.sqrt(sq / len(samples)):.1f}')

    if span < 0.3:
        print('注意：角度范围小于 0.3 rad，offset 不可靠，建议扩大采样范围。')
    if any(abs(out) >= TOTAL_MAX_OUT * 0.99 for _, out in samples):
        print('注意：有采样点的输出已到总限幅 30000，该点不是真实保持输出，应剔除。')
    if abs(gain) > GRAVITY_MAX_OUT:
        print(f'注意：gain 超过前馈限幅 {GRAVITY_MAX_OUT:.0f}，需要先调大 PITCH_GRAVITY_MAX_OUT，'
              '否则部分角度的补偿会被截断。')
    print()
    print('在调试器里填入 pitch_gravity_config.gain 和 .offset_rad 验证；'
          '确认后写回 PITCH_GRAVITY_GAIN_DEFAULT / PITCH_GRAVITY_OFFSET_DEFAULT。')


if __name__ == '__main__':
    main()

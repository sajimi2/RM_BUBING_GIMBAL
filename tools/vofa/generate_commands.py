"""生成 VOFA+ 可载入的命令表；格式依据本机 VOFA+ 保存的 cmds JSON。"""
from pathlib import Path
import json

COMMANDS = [
    ('重力 Gain', 'gain:%f\n'),
    ('重力 Offset（度）', 'offset:%f\n'),
    ('Pitch 速度环 Ki', 'ki:%f\n'),
    ('模拟 Pitch 目标（度）', 'target:%f\n'),
    ('进入串口模拟并保持当前角度', 'source:1\n'),
    ('恢复 CAN 视觉', 'source:0\n'),
    ('读取当前状态', 'read:0\n'),
    ('暂时关闭重力补偿', 'gravity:0\n'),
    ('恢复关闭前重力参数', 'gravity:1\n'),
    ('上传周期 20 ms', 'stream:20\n'),
    ('暂停周期上传', 'stream:0\n'),
    ('记录组号', 'mark:%d\n'),
    ('关闭积分并清零 I 项', 'ki:0\n'),
]


def command(name, text, group=False):
    return {
        'name': name, 'intro_on': True, 'intro': '', 'hex_on': False,
        'loop_on': False, 'loop_ms': 100, 'loop_count': 1,
        'cmd_hex': text.encode('ascii').hex(' ').upper(),
        'is_group': group, 'down': group, 'subCmds': [],
    }


def main():
    # 只生成工程文件；不覆盖用户的 VOFA 配置，也不连接串口。
    root = command('云台调参', '', True)
    root['subCmds'] = [command(name, text) for name, text in COMMANDS]
    output = Path(__file__).with_name('pitch-commands.json')
    output.write_text(json.dumps({'type': 'cmds', 'vnumber': 100, 'ctx': root},
                                 ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()

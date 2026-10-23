"""离线检查CSV截取边界、中文编码、原件保护及Tk窗口；不连接串口。"""
from pathlib import Path
import csv
import importlib.util
import tempfile
import tkinter as tk

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('vofa_csv_trim', ROOT / 'tools/vofa_csv_trim.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

with tempfile.TemporaryDirectory() as temporary:
    folder = Path(temporary)
    source = folder / '原始.csv'
    original = '实际角,目标角\r\n0,0\r\n1.234567,2\r\n3,4\r\n5,6\r\n'.encode('gb18030')
    source.write_bytes(original)
    header, rows, delimiter = module.load_csv(source)
    assert header == ['实际角', '目标角'] and len(rows) == 4
    output = folder / '片段.csv'
    assert module.save_slice(source, output, header, rows, 2, 3, delimiter) == 2
    new_header, selected, _ = module.load_csv(output)
    assert new_header == header and selected == rows[1:3]
    assert source.read_bytes() == original
    for first, last in [(0, 2), (3, 2), (1, 5)]:
        try:
            module.save_slice(source, output, header, rows, first, last)
            raise AssertionError('Invalid range accepted')
        except ValueError:
            pass
    try:
        module.save_slice(source, source, header, rows, 1, 4)
        raise AssertionError('Original file overwritten')
    except ValueError:
        pass
    root = tk.Tk()
    root.withdraw()
    app = module.TrimWindow(root)
    app.header, app.rows = header, rows
    for box in (app.channel_a, app.channel_b):
        box['values'] = header
        box.current(0)
    app.first.set('1')
    app.last.set('4')
    root.update_idletasks()
    app.draw()
    assert app.row_at(-100) == 1 and app.row_at(100000) == 4
    app.select_recent()
    assert app.selection() == (1, 4)
    root.destroy()

print('PASS: CSV inclusive range/GB18030 and UTF-8 headers/zero preservation/original protection/Tk window')

"""离线截取VOFA CSV；只选择数据行，不连接串口，不改原文件。"""
from __future__ import annotations

import csv
import io
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


def load_csv(path: Path):
    raw = path.read_bytes()
    try:
        text = raw.decode('utf-8-sig')
    except UnicodeDecodeError:
        text = raw.decode('gb18030')
    try:
        delimiter = csv.Sniffer().sniff(text[:8192], delimiters=',;\t').delimiter
    except csv.Error:
        delimiter = ','
    records = list(csv.reader(io.StringIO(text), delimiter=delimiter))
    if len(records) < 2:
        raise ValueError('文件至少需要一行表头和一行数据')
    if not records[0]:
        raise ValueError('CSV表头为空')
    return records[0], records[1:], delimiter


def save_slice(source: Path, destination: Path, header, rows, first: int, last: int, delimiter=','):
    if source.resolve() == destination.resolve():
        raise ValueError('请选择新文件，不能覆盖原始记录')
    if not 1 <= first <= last <= len(rows):
        raise ValueError(f'范围应为1～{len(rows)}，且开始不大于结束')
    # 行号从1开始，首尾都保留；不丢零值、不排序、不平滑。
    with destination.open('w', encoding='utf-8-sig', newline='') as output:
        writer = csv.writer(output, delimiter=delimiter)
        writer.writerow(header)
        writer.writerows(rows[first - 1:last])
    return last - first + 1


class TrimWindow:
    def __init__(self, root):
        self.root = root
        self.source = None
        self.header = []
        self.rows = []
        self.delimiter = ','
        self.drag_start = None
        self.first = tk.StringVar(value='1')
        self.last = tk.StringVar(value='1')
        self.period = tk.StringVar(value='20')
        self.message = tk.StringVar(value='打开CSV，在图上拖选一段，再另存为新CSV。')
        root.title('VOFA CSV 截取')
        root.geometry('1060x650')
        root.minsize(780, 500)
        frame = ttk.Frame(root, padding=14)
        frame.pack(fill='both', expand=True)
        top = ttk.Frame(frame)
        top.pack(fill='x')
        ttk.Button(top, text='打开 CSV', command=self.open_file).pack(side='left')
        ttk.Label(top, textvariable=self.message, wraplength=840).pack(side='left', padx=12)
        controls = ttk.Frame(frame, padding=(0, 12))
        controls.pack(fill='x')
        for label, variable in [('开始数据行', self.first), ('结束数据行', self.last), ('估算周期 ms', self.period)]:
            ttk.Label(controls, text=label).pack(side='left', padx=(0, 4))
            ttk.Entry(controls, textvariable=variable, width=9).pack(side='left', padx=(0, 14))
        ttk.Button(controls, text='显示选区', command=self.draw).pack(side='left')
        ttk.Button(controls, text='最后10秒', command=self.select_recent).pack(side='left', padx=6)
        ttk.Button(controls, text='另存选中范围', command=self.save).pack(side='right')
        channels = ttk.Frame(frame)
        channels.pack(fill='x')
        ttk.Label(channels, text='预览通道 A / B：').pack(side='left')
        self.channel_a = ttk.Combobox(channels, state='readonly', width=30)
        self.channel_b = ttk.Combobox(channels, state='readonly', width=30)
        self.channel_a.pack(side='left', padx=5)
        self.channel_b.pack(side='left', padx=5)
        self.channel_a.bind('<<ComboboxSelected>>', self.draw)
        self.channel_b.bind('<<ComboboxSelected>>', self.draw)
        self.canvas = tk.Canvas(frame, background='white', highlightthickness=1, highlightbackground='#c7d3df')
        self.canvas.pack(fill='both', expand=True, pady=10)
        self.canvas.bind('<Configure>', self.draw)
        self.canvas.bind('<ButtonPress-1>', self.begin_selection)
        self.canvas.bind('<B1-Motion>', self.drag_selection)
        self.canvas.bind('<ButtonRelease-1>', self.drag_selection)
        ttk.Label(frame, text='横轴：数据行号（不含表头）。拖动选择起止位置，也可直接改行号。预览降采样，导出保留选区全部原始行。').pack(anchor='w')
        ttk.Label(frame, text='秒数按输入周期估算；不代表硬件时间戳。工具不自动删除零值、旧格式或异常行，原文件始终保留。').pack(anchor='w')

    def open_file(self):
        name = filedialog.askopenfilename(title='打开VOFA导出的CSV', filetypes=[('CSV', '*.csv'), ('所有文件', '*.*')])
        if not name:
            return
        try:
            header, rows, delimiter = load_csv(Path(name))
        except (OSError, UnicodeError, ValueError, csv.Error) as error:
            messagebox.showerror('无法打开', str(error))
            return
        self.source = Path(name)
        self.header, self.rows, self.delimiter = header, rows, delimiter
        self.first.set('1')
        self.last.set(str(len(rows)))
        labels = [f'{index}: {name}' for index, name in enumerate(header)]
        for box in (self.channel_a, self.channel_b):
            box['values'] = labels
        # 当前8通道表的Yaw实际/目标位于第3、第4列。
        self.channel_a.current(min(2, len(header) - 1))
        self.channel_b.current(min(3, len(header) - 1))
        self.message.set(f'{self.source.name} · {len(rows)}行；拖选后另存，不改原件')
        self.draw()

    def selection(self):
        first, last = int(self.first.get()), int(self.last.get())
        if not 1 <= first <= last <= len(self.rows):
            raise ValueError(f'范围应为1～{len(self.rows)}，开始不能大于结束')
        return first, last

    def select_recent(self):
        if not self.rows:
            return
        try:
            period = float(self.period.get())
            if not math.isfinite(period) or period <= 0:
                raise ValueError('周期必须是正数')
            count = max(1, math.ceil(10000 / period))
            self.first.set(str(max(1, len(self.rows) - count + 1)))
            self.last.set(str(len(self.rows)))
            self.draw()
        except ValueError as error:
            messagebox.showerror('周期无效', str(error))

    def row_at(self, x):
        width = max(1, self.canvas.winfo_width() - 70)
        fraction = max(0, min(1, (x - 55) / width))
        return 1 + round(fraction * (len(self.rows) - 1))

    def begin_selection(self, event):
        if self.rows:
            self.drag_start = self.row_at(event.x)
            self.drag_selection(event)

    def drag_selection(self, event):
        if self.drag_start is None:
            return
        end = self.row_at(event.x)
        self.first.set(str(min(self.drag_start, end)))
        self.last.set(str(max(self.drag_start, end)))
        self.draw()

    def draw(self, event=None):
        self.canvas.delete('all')
        if not self.rows:
            return
        width = max(1, self.canvas.winfo_width() - 70)
        height = max(1, self.canvas.winfo_height() - 50)
        # 只简化预览绘制，CSV里的数据不经过这里的抽点过程。
        step = max(1, len(self.rows) // max(1, int(width)))
        samples = list(range(0, len(self.rows), step))
        if samples[-1] != len(self.rows) - 1:
            samples.append(len(self.rows) - 1)
        curves = []
        for box in (self.channel_a, self.channel_b):
            column = box.current()
            points = []
            for index in samples:
                try:
                    value = float(self.rows[index][column])
                    if math.isfinite(value):
                        points.append((index, value))
                except (ValueError, IndexError):
                    pass
            curves.append(points)
        values = [value for points in curves for _, value in points]
        if not values:
            self.canvas.create_text(80, 30, anchor='nw', text='所选通道没有可预览的数值')
            return
        lower, upper = min(values), max(values)
        margin = max(0.1, (upper - lower) * 0.08)
        lower, upper = lower - margin, upper + margin
        self.canvas.create_text(50, 20, anchor='e', text=f'{upper:.2f}')
        self.canvas.create_text(50, height + 20, anchor='e', text=f'{lower:.2f}')
        self.canvas.create_text(55, height + 35, anchor='w', text='1')
        self.canvas.create_text(55 + width, height + 35, anchor='e', text=str(len(self.rows)))
        try:
            first, last = self.selection()
            x1 = 55 + (first - 1) * width / max(1, len(self.rows) - 1)
            x2 = 55 + (last - 1) * width / max(1, len(self.rows) - 1)
            self.canvas.create_rectangle(x1, 20, max(x1 + 1, x2), height + 20, fill='#e5f1fc', outline='#64a5dc')
        except ValueError:
            pass
        for points, color in zip(curves, ('#176eb3', '#e38319')):
            coordinates = []
            for index, value in points:
                coordinates.extend((55 + index * width / max(1, len(self.rows) - 1),
                                    20 + height * (upper - value) / (upper - lower)))
            if len(coordinates) >= 4:
                self.canvas.create_line(*coordinates, fill=color, width=1.5)

    def save(self):
        if self.source is None:
            return
        try:
            first, last = self.selection()
        except ValueError as error:
            messagebox.showerror('范围无效', str(error))
            return
        name = filedialog.asksaveasfilename(title='另存截取结果', initialdir=self.source.parent,
                                          initialfile=f'{self.source.stem}_{first}-{last}.csv',
                                          defaultextension='.csv', filetypes=[('CSV', '*.csv')])
        if not name:
            return
        try:
            count = save_slice(self.source, Path(name), self.header, self.rows, first, last, self.delimiter)
        except (OSError, ValueError) as error:
            messagebox.showerror('无法保存', str(error))
            return
        self.message.set(f'已保存{count}行：{Path(name).name}，原文件未改动')


if __name__ == '__main__':
    window = tk.Tk()
    TrimWindow(window)
    window.mainloop()

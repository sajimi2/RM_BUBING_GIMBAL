"""本机调参台：浏览器界面 + 单工作线程串口；不修改固件或自动连接设备。"""
from __future__ import annotations

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import queue
import secrets
import threading
import time
from urllib.parse import parse_qs, urlsplit
import urllib.request
import webbrowser

from tuning_link import TuningLink, STATE_FIELDS, FLAG_FIELDS

ROOT = Path(__file__).resolve().parents[1]
WEB = Path(__file__).resolve().parent / 'tuning_web'
APP_ID = 'gimbal-tuning-panel-v1'
CSV_FIELDS = ['received_at', 'port', 'tick_ms', 'source', 'rc_online', 'automatic',
              *STATE_FIELDS, 'motor_cmd', 'speed_ki', *FLAG_FIELDS]


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name}必须是有效数字')
    return float(value)


def clean_state(state):
    # 无效的传感器值显示为空，不允许 NaN 变成 JSON 或误显示为零。
    return {key: None if isinstance(value, float) and not math.isfinite(value) else value
            for key, value in state.items()}


def csv_text(rows):
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction='ignore')
    writer.writeheader()
    writer.writerows(rows)
    return '\ufeff' + output.getvalue()


class PanelController:
    """只有 worker 使用串口。HTTP 线程只投递操作/读取快照，浏览器不会被串口等待卡住。"""
    def __init__(self, connector=TuningLink.connect, interval=0.2):
        self.connector = connector
        self.interval = interval
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=12)
        self.stop_event = threading.Event()
        self.link = None
        self.connected = False
        self.port = ''
        self.info = None
        self.parameters = []
        self.state = None
        self.updated = 0.0
        self.error = ''
        self.note = '选择 DAP 串口并连接；连接不会自动使能或发送目标。'
        self.samples = deque(maxlen=1500)  # 最近五分钟用于曲线。
        self.sequence = 0
        self.records = []
        self.recording = False
        self.recorded = []  # 显式开始记录后保存，最多 18000 点（约一小时）。
        self.previous = None
        self.jobs = {}
        self.next_job = 0
        self.thread = threading.Thread(target=self._run, name='gimbal-serial', daemon=True)
        self.thread.start()

    def submit(self, action, values=None):
        if action not in {'connect', 'disconnect', 'parameters', 'zero', 'restore',
                          'uart', 'can', 'target', 'read', 'mark', 'record_start', 'record_stop'}:
            raise ValueError('未知操作')
        with self.lock:
            self.next_job += 1
            identifier = self.next_job
            self.jobs[identifier] = {'done': False}
            while len(self.jobs) > 128:
                oldest = next(iter(self.jobs))
                if not self.jobs[oldest]['done']:
                    raise ValueError('操作过多，请等待当前操作完成')
                del self.jobs[oldest]
        try:
            self.commands.put_nowait((identifier, action, values or {}, time.monotonic()))
        except queue.Full:
            with self.lock:
                self.jobs[identifier] = {'done': True, 'ok': False, 'error': '操作队列已满'}
        return identifier

    def job(self, identifier):
        with self.lock:
            return dict(self.jobs.get(identifier, {'done': True, 'ok': False, 'error': '操作已过期'}))

    def view(self, since=0):
        with self.lock:
            return dict(connected=self.connected, port=self.port, info=self.info,
                        parameters=self.parameters, state=self.state,
                        age_ms=None if not self.updated else int((time.monotonic() - self.updated) * 1000),
                        error=self.error, note=self.note, can_restore=self.previous is not None,
                        recording=self.recording, recorded_count=len(self.recorded),
                        records=list(self.records),
                        samples=[sample for sample in self.samples if sample['seq'] > since][-300:])

    def _sample(self, state):
        state = clean_state(state)
        with self.lock:
            self.state = state
            self.updated = time.monotonic()
            self.sequence += 1
            row = dict(state, seq=self.sequence, received_s=self.updated,
                       received_at=datetime.now(timezone.utc).isoformat(timespec='milliseconds'), port=self.port)
            self.samples.append(row)
            if self.recording:
                self.recorded.append(row)
                if len(self.recorded) >= 18000:
                    self.recording = False
                    self.note = '记录已达到 18000 点，已停止。请导出 CSV。'
        return state

    def _read(self):
        return self._sample(self.link.state())

    def _close(self, restore):
        note = '串口已断开。'
        if self.link:
            try:
                if restore:
                    self.link.select_source(False)
                    if self.link.state()['source'] != 0:
                        raise ValueError('CAN 来源未确认')
                    note = '已确认恢复 CAN 来源，并断开串口。'
            except Exception as error:
                note = f'串口已关闭，但未确认恢复 CAN：{error}。目标可能保持，请用遥控器退出自瞄。'
            finally:
                try:
                    self.link.close()
                except Exception as error:
                    note += f' 关闭端口时发生错误：{error}'
                finally:
                    self.link = None
        with self.lock:
            self.connected = False
            self.recording = False
            self.previous = None
            self.note = note
        return note

    def _connect(self, values):
        if self.link:
            raise ValueError('请先断开当前设备')
        port = values.get('port')
        if not isinstance(port, str) or not port.strip() or len(port) > 100:
            raise ValueError('请选择串口')
        self.link = self.connector(port.strip())
        try:
            info = self.link.hello()
            for key in ('pitch_min_rad', 'pitch_max_rad', 'gain_max', 'offset_max_rad'):
                finite(info[key], key)
            parameters = self.link.parameters()
            # 本界面只暴露这两个参数；先验证设备注册表，防止误连其他固件。
            names = {p['id']: p['name'] for p in parameters}
            if names.get(1) != 'pitch.gravity.gain' or names.get(2) != 'pitch.gravity.offset':
                raise ValueError('设备参数表不匹配，请烧录本工程的串口调参固件')
            state = self.link.state()
        except Exception:
            self._close(False)
            raise
        with self.lock:
            self.port = port.strip()
            self.info = info
            self.parameters = parameters
            self.samples.clear()
            self.previous = None
            self.connected = True
            self.error = ''
            self.note = '握手成功，正在读取板端状态。尚未改变参数或目标来源。'
        self._sample(state)
        return self.note

    def _set_parameters(self, gain, offset):
        gain, offset = finite(gain, 'Gain'), finite(offset, 'Offset')
        if abs(gain) > self.info['gain_max'] or abs(offset) > self.info['offset_max_rad']:
            raise ValueError('参数超出板端允许范围')
        confirmed = self.link.set_parameters({1: gain, 2: offset})
        current = self._read()
        if any(current[key] is None or not math.isclose(current[key], confirmed[identifier], rel_tol=1e-5, abs_tol=1e-6)
               for identifier, key in ((1, 'gain'), (2, 'offset_rad'))):
            raise ValueError('写入已回复，但当前读回值不同；请以板端读回为准')
        return '参数已写入 RAM 并读回确认；重启恢复固件默认值。'

    def _execute(self, action, values):
        if action == 'connect':
            return self._connect(values)
        if action == 'disconnect':
            return self._close(True)
        if action == 'record_stop':
            with self.lock:
                self.recording = False
            return '已停止记录，CSV 保留在本次调参台会话中。'
        if not self.link:
            raise ValueError('请先连接设备')
        if action == 'parameters':
            return self._set_parameters(values.get('gain'), math.radians(finite(values.get('offset_deg'), 'Offset')))
        if action == 'zero':
            state = self._read()
            if state['gain'] != 0:
                previous = (state['gain'], state['offset_rad'])
            else:
                previous = self.previous
            result = self._set_parameters(0, state['offset_rad'])
            with self.lock:
                self.previous = previous
            return result
        if action == 'restore':
            if self.previous is None:
                raise ValueError('本次连接尚未保存关闭补偿前的参数')
            return self._set_parameters(*self.previous)
        if action in ('uart', 'target'):
            state = self._read()
            if not state['rc_online'] or not state['automatic'] or not state['control_enabled']:
                raise ValueError('请打开遥控器、解除无力并进入自瞄；串口不能代替遥控器使能')
            if action == 'uart':
                target = finite(state['actual_angle_rad'], '实际角度')
            else:
                if state['source'] != 1:
                    raise ValueError('请先点击“开始串口模拟”')
                target = math.radians(finite(values.get('target_deg'), '目标角度'))
            if not self.info['pitch_min_rad'] <= target <= self.info['pitch_max_rad']:
                raise ValueError('目标超出板端允许范围；先用遥控器移回允许范围')
            if action == 'uart':
                self.link.select_source(True)
            self.link.set_target(target)
            self._read()
            return '目标帧已确认接收，请看实际角度判断是否到位。' if action == 'target' else '已选择串口模拟，并发送当前实际角度作为初始目标。'
        if action == 'can':
            self.link.select_source(False)
            if self._read()['source'] != 0:
                raise ValueError('未确认恢复 CAN 来源')
            return '已确认恢复 CAN 视觉来源。'
        if action == 'read':
            self._read()
            return '已读取最新板端状态。'
        if action == 'mark':
            self._read()
            with self.lock:
                if len(self.records) >= 500:
                    raise ValueError('对照快照已达 500 组，请导出后重新启动调参台')
                self.records.append(dict(self.samples[-1]))
            return '已保存一组实际读数，可导出快照 CSV。'
        if action == 'record_start':
            with self.lock:
                if self.recording:
                    raise ValueError('正在记录，请先停止当前记录')
                self.recorded = []
                self.recording = True
            return '已开始新记录，上一轮连续记录已清空；CSV 只导出从现在开始的读数。'
        raise ValueError('未知操作')

    def _run(self):
        next_read = 0.0
        while not self.stop_event.is_set():
            try:
                identifier, action, values, submitted = self.commands.get(timeout=0.04)
            except queue.Empty:
                identifier = None
            if identifier is not None:
                try:
                    if time.monotonic() - submitted > 6:
                        raise ValueError('操作等待过久，已取消；请重新检查状态后点击')
                    note = self._execute(action, values)
                    with self.lock:
                        self.note = note
                        self.error = ''
                        self.jobs[identifier] = dict(done=True, ok=True, message=note)
                except Exception as error:
                    # 写入失败不自动重试；通信失败后释放端口，保留最后读数和记录。
                    message = str(error)
                    if not isinstance(error, ValueError):
                        self._close(False)
                        message += '。操作结果未确认，连接已关闭，不自动重发；目标可能保持，请用遥控器退出自瞄。'
                    elif self.link:
                        try:
                            self._read()
                        except Exception:
                            self._close(False)
                    with self.lock:
                        self.error = message
                        self.jobs[identifier] = dict(done=True, ok=False, error=message)
                next_read = time.monotonic() + self.interval
            if self.link and time.monotonic() >= next_read:
                try:
                    self._read()
                except Exception as error:
                    self._close(False)
                    with self.lock:
                        self.error = f'通信中断：{error}。目标可能保持，请用遥控器退出自瞄。'
                next_read = time.monotonic() + self.interval
        self._close(True)

    def csv(self, snapshots=False):
        with self.lock:
            return csv_text(list(self.records if snapshots else self.recorded))

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=5)


class PanelServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, controller):
        super().__init__(address, PanelHandler)
        self.controller = controller
        self.token = secrets.token_hex(24)


class PanelHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def reply(self, code, data, content_type='application/json; charset=utf-8', filename=None):
        if isinstance(data, (dict, list)):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        body = data.encode('utf-8') if isinstance(data, str) else data
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'")
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def valid_host(self):
        return self.headers.get('Host') == f'127.0.0.1:{self.server.server_port}'

    def do_GET(self):
        if not self.valid_host():
            return self.reply(403, {'error': '仅允许本机访问'})
        parsed = urlsplit(self.path)
        path = parsed.path
        try:
            if path == '/api/health':
                return self.reply(200, {'app': APP_ID})
            if path == '/favicon.ico':
                return self.reply(204, b'', 'image/x-icon')
            if path == '/api/status':
                since = int(parse_qs(parsed.query).get('since', ['0'])[0])
                return self.reply(200, self.server.controller.view(since))
            if path == '/api/ports':
                from serial.tools import list_ports
                ports = [dict(device=p.device, description=p.description, bluetooth=(p.hwid or '').startswith('BTHENUM'))
                         for p in list_ports.comports()]
                return self.reply(200, sorted(ports, key=lambda p: (p['bluetooth'], p['device'])))
            if path.startswith('/api/job/'):
                return self.reply(200, self.server.controller.job(int(path.rsplit('/', 1)[1])))
            if path in ('/recording.csv', '/snapshots.csv'):
                snapshots = path == '/snapshots.csv'
                return self.reply(200, self.server.controller.csv(snapshots), 'text/csv; charset=utf-8',
                                  'gimbal-snapshots.csv' if snapshots else 'gimbal-recording.csv')
            assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/panel.js': ('panel.js', 'text/javascript; charset=utf-8'),
                      '/panel.css': ('panel.css', 'text/css; charset=utf-8')}
            if path in assets:
                name, kind = assets[path]
                data = (WEB / name).read_text(encoding='utf-8')
                if path == '/':
                    data = data.replace('__PANEL_TOKEN__', self.server.token)
                return self.reply(200, data, kind)
            self.reply(404, {'error': '页面不存在'})
        except Exception as error:
            self.reply(400, {'error': str(error)})

    def do_POST(self):
        origin = self.headers.get('Origin')
        expected = f'http://127.0.0.1:{self.server.server_port}'
        if (not self.valid_host() or origin not in (None, expected) or
                self.headers.get('X-Panel-Token') != self.server.token or
                self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
            return self.reply(403, {'error': '请求来源无效，请从本机调参台页面操作'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 2048:
                raise ValueError('请求长度无效')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求格式无效')
            if self.path == '/api/action':
                identifier = self.server.controller.submit(data.get('action'), data)
                return self.reply(202, {'job': identifier})
            if self.path == '/api/shutdown':
                # shutdown 必须由其他线程调用，关闭时 worker 尝试恢复 CAN，失败则不宣称成功。
                note = self.server.controller.submit('disconnect')
                deadline = time.monotonic() + 5
                while not self.server.controller.job(note)['done'] and time.monotonic() < deadline:
                    time.sleep(0.05)
                job = self.server.controller.job(note)
                if not job['done'] or not job.get('ok'):
                    raise ValueError('断开尚未完成，请先用遥控器退出自瞄，再点击断开')
                self.reply(200, {'message': job.get('message', '已退出')})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            self.reply(404, {'error': '未知操作'})
        except Exception as error:
            self.reply(400, {'error': str(error)})


def main():
    parser = argparse.ArgumentParser(description='云台调参台（仅本机）')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    import serial  # 启动时发现缺依赖，不把错误留到现场点击连接时。
    del serial
    url = f'http://127.0.0.1:{args.port}'
    try:
        with urllib.request.urlopen(url + '/api/health', timeout=0.5) as response:
            running = json.load(response).get('app') == APP_ID
    except Exception:
        running = False
    if running:
        if not args.no_browser:
            webbrowser.open(url)
        return
    controller = PanelController()
    try:
        server = PanelServer(('127.0.0.1', args.port), controller)
        if not args.no_browser:
            webbrowser.open(url)
        server.serve_forever(poll_interval=0.2)
    finally:
        controller.stop()
        if 'server' in locals():
            server.server_close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        # pythonw 无控制台；启动失败时仍给用户一个可见原因。
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, f'调参台启动失败：{error}\n请确认 Python 和 pyserial 已安装。', '云台调参台', 0x10)
        raise

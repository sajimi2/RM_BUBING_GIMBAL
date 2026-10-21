"""测试图形调参台业务和 HTTP；测试夹具不打开串口、不模拟真实机械响应。"""
from pathlib import Path
import csv
import io
import json
import math
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from tuning_panel import PanelController, PanelServer, PanelHandler, APP_ID
from tuning_link import STATE_FIELDS, FLAG_FIELDS


class FixtureLink:
    def __init__(self):
        self.calls = []
        self.closed = False
        self.fail = False
        self.legacy = False
        self.values = {key: 0.0 for key in STATE_FIELDS}
        self.values.update({key: False for key in FLAG_FIELDS})
        self.values.update(gain=-800.0, target_angle_rad=0.04, requested_angle_rad=0.04,
                           actual_angle_rad=0.02, angle_error_rad=0.02, p_out=1200.0,
                           pid_unclamped_out=1200.0, gravity_target_out=-800.0, gravity_out=-800.0,
                           total_unclamped_out=400.0, total_out=400.0, motor_cmd=-400, speed_ki=0.0,
                           rc_online=1, automatic=1, control_enabled=True, source=0, tick_ms=1234)

    def hello(self):
        return dict(protocol=1, parameter_count=2 if self.legacy else 3, device='gimbal-pitch', pitch_min_rad=-0.45,
                    pitch_max_rad=0.7, gain_max=5000.0, offset_max_rad=math.pi,
                    gravity_max_out=5000.0, total_max_out=24000.0)

    def parameters(self):
        gravity = [dict(id=1, name='pitch.gravity.gain'), dict(id=2, name='pitch.gravity.offset')]
        return gravity if self.legacy else gravity + [dict(id=3, name='pitch.speed.ki', minimum=0, maximum=8000, step=100)]

    def state(self):
        if self.fail:
            raise TimeoutError('fixture connection lost')
        return dict(self.values)

    def set_parameters(self, values):
        self.calls.append(('parameters', dict(values)))
        for identifier, value in values.items():
            self.values[{1:'gain', 2:'offset_rad', 3:'speed_ki'}[identifier]] = value
        if values.get(3) == 0:
            self.values['i_out'] = 0
        return dict(values)

    def select_source(self, uart):
        self.calls.append(('source', uart))
        self.values['source'] = int(uart)

    def set_target(self, target):
        self.calls.append(('target', target))
        self.values.update(requested_angle_rad=target, target_angle_rad=target,
                           angle_error_rad=target-self.values['actual_angle_rad'])

    def close(self):
        self.closed = True


def wait_job(controller, action, **values):
    identifier = controller.submit(action, values)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        job = controller.job(identifier)
        if job['done']:
            return job
        time.sleep(0.01)
    raise AssertionError('worker did not finish')


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.fixture = FixtureLink()
        self.opened = []
        def connector(port):
            self.opened.append(port)
            return self.fixture
        self.panel = PanelController(connector, interval=0.02)

    def tearDown(self):
        self.panel.stop()

    def connect(self):
        self.assertTrue(wait_job(self.panel, 'connect', port='TEST')['ok'])

    def test_connect_only_reads_and_write_requires_explicit_action(self):
        self.assertEqual(self.opened, [])
        self.connect()
        self.assertEqual(self.fixture.calls, [])
        self.assertTrue(wait_job(self.panel, 'parameters', gain=-900.0, offset_deg=10.0)['ok'])
        state = self.panel.view()['state']
        self.assertEqual(state['gain'], -900)
        self.assertAlmostEqual(state['offset_rad'], math.radians(10))
        self.assertEqual(state['speed_ki'], 0)
        count = len(self.fixture.calls)
        for values in (dict(gain=float('nan'), offset_deg=0), dict(gain=-5001, offset_deg=0),
                       dict(gain=0, offset_deg=181), dict(gain=True, offset_deg=0)):
            self.assertFalse(wait_job(self.panel, 'parameters', **values)['ok'])
        self.assertEqual(len(self.fixture.calls), count)

    def test_ki_write_is_independent_validated_read_back_and_recorded(self):
        self.connect()
        self.fixture.values['i_out'] = 321
        self.assertTrue(wait_job(self.panel, 'ki', speed_ki=500)['ok'])
        self.assertEqual(self.fixture.calls, [('parameters', {3:500})])
        self.assertEqual(self.panel.view()['state']['speed_ki'], 500)
        self.assertEqual(self.panel.view()['state']['i_out'], 321)
        self.assertEqual(self.panel.view()['state']['gain'], -800)
        self.assertEqual(self.panel.view()['state']['offset_rad'], 0)
        for value in (-1, 8001, float('nan'), float('inf'), True, '500', None):
            self.assertFalse(wait_job(self.panel, 'ki', speed_ki=value)['ok'])
        self.assertEqual(len(self.fixture.calls), 1)
        self.assertTrue(wait_job(self.panel, 'record_start')['ok'])
        self.assertTrue(wait_job(self.panel, 'read')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_stop')['ok'])
        rows = list(csv.DictReader(io.StringIO(self.panel.csv().lstrip('\ufeff'))))
        self.assertTrue(all(float(row['speed_ki']) == 500 for row in rows))
        self.assertTrue(wait_job(self.panel, 'ki', speed_ki=0)['ok'])
        self.assertEqual(self.panel.view()['state']['i_out'], 0)
        self.assertEqual(self.panel.view()['state']['speed_ki'], 0)
        self.assertEqual(self.panel.view()['state']['source'], 0)

    def test_legacy_firmware_can_connect_but_cannot_write_ki(self):
        self.fixture.legacy = True
        self.connect()
        result = wait_job(self.panel, 'ki', speed_ki=500)
        self.assertFalse(result['ok'])
        self.assertIn('烧录新版', result['error'])
        self.assertEqual(self.fixture.calls, [])
        self.assertTrue(self.panel.view()['connected'])

    def test_ki_readback_mismatch_is_not_reported_as_success(self):
        self.connect()
        self.fixture.set_parameters = lambda values: dict(values)
        result = wait_job(self.panel, 'ki', speed_ki=500)
        self.assertFalse(result['ok'])
        self.assertIn('读回值不同', result['error'])
        self.assertEqual(self.panel.view()['state']['speed_ki'], 0)

    def test_zero_restore_and_uart_gates(self):
        self.connect()
        self.assertTrue(wait_job(self.panel, 'zero')['ok'])
        self.assertEqual(self.panel.view()['state']['gain'], 0)
        self.assertTrue(wait_job(self.panel, 'restore')['ok'])
        self.assertEqual(self.panel.view()['state']['gain'], -800)
        self.assertFalse(wait_job(self.panel, 'target', target_deg=5)['ok'])
        for key in ('rc_online', 'automatic', 'control_enabled'):
            self.fixture.values[key] = 0
            self.assertFalse(wait_job(self.panel, 'uart')['ok'])
            self.fixture.values[key] = 1
        self.assertNotIn(('source', True), self.fixture.calls)
        self.assertTrue(wait_job(self.panel, 'uart')['ok'])
        self.assertEqual(self.fixture.calls[-2:], [('source', True), ('target', 0.02)])
        self.assertTrue(wait_job(self.panel, 'target', target_deg=5)['ok'])
        self.assertAlmostEqual(self.fixture.calls[-1][1], math.radians(5))
        self.assertFalse(wait_job(self.panel, 'target', target_deg=80)['ok'])
        self.assertTrue(wait_job(self.panel, 'can')['ok'])
        self.assertEqual(self.panel.view()['state']['source'], 0)

    def test_records_csv_and_disconnect_confirm(self):
        self.connect()
        self.assertTrue(wait_job(self.panel, 'mark')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_start')['ok'])
        self.assertTrue(wait_job(self.panel, 'read')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_stop')['ok'])
        rows = list(csv.DictReader(io.StringIO(self.panel.csv().lstrip('\ufeff'))))
        self.assertGreater(len(rows), 0)
        self.assertEqual(float(rows[0]['gain']), -800)
        self.assertEqual(rows[0]['port'], 'TEST')
        snapshots = list(csv.DictReader(io.StringIO(self.panel.csv(True).lstrip('\ufeff'))))
        self.assertEqual(len(snapshots), 1)
        result = wait_job(self.panel, 'disconnect')
        self.assertTrue(result['ok'])
        self.assertIn('已确认', result['message'])
        self.assertTrue(self.fixture.closed)
        self.assertFalse(self.panel.view()['connected'])
        self.assertEqual(len(self.panel.view()['records']), 1)

    def test_lost_connection_preserves_last_state_and_does_not_retry_writes(self):
        self.connect()
        self.fixture.fail = True
        result = wait_job(self.panel, 'parameters', gain=-1000, offset_deg=0)
        self.assertFalse(result['ok'])
        self.assertFalse(self.panel.view()['connected'])
        self.assertEqual(len([c for c in self.fixture.calls if c[0] == 'parameters']), 1)
        self.assertEqual(self.panel.view()['state']['gain'], -800)
        self.assertIsNotNone(self.panel.view()['age_ms'])
        self.assertFalse(wait_job(self.panel, 'target', target_deg=5)['ok'])

    def test_new_record_excludes_previous_round_without_changing_device(self):
        self.connect()
        self.assertTrue(wait_job(self.panel, 'mark')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_start')['ok'])
        self.assertTrue(wait_job(self.panel, 'read')['ok'])
        old_rows = list(self.panel.recorded)
        self.assertGreater(len(old_rows), 0)
        self.assertFalse(wait_job(self.panel, 'record_start')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_stop')['ok'])
        self.fixture.values.update(gain=1300.0, tick_ms=5678)
        self.assertTrue(wait_job(self.panel, 'record_start')['ok'])
        self.assertTrue(wait_job(self.panel, 'read')['ok'])
        self.assertTrue(wait_job(self.panel, 'record_stop')['ok'])
        rows = list(csv.DictReader(io.StringIO(self.panel.csv().lstrip('\ufeff'))))
        self.assertTrue(rows)
        self.assertTrue(all(float(row['gain']) == 1300 and float(row['tick_ms']) == 5678 for row in rows))
        self.assertFalse({row['seq'] for row in self.panel.recorded} & {row['seq'] for row in old_rows})
        self.assertEqual(len(self.panel.view()['records']), 1)
        self.assertEqual(self.fixture.calls, [])
        # 达到上一轮上限也能直接开始新一轮。
        with self.panel.lock:
            self.panel.recorded = [old_rows[0]] * 18000
        self.assertTrue(wait_job(self.panel, 'record_start')['ok'])
        self.assertLess(self.panel.view()['recorded_count'], 18000)

    def test_http_serves_panel_and_rejects_untrusted_write(self):
        server = PanelServer(('127.0.0.1', 0), self.panel)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{server.server_port}'
        try:
            with urllib.request.urlopen(url + '/') as response:
                page = response.read().decode('utf-8')
                self.assertIn(server.token, page)
                self.assertNotIn('__PANEL_TOKEN__', page)
            request = urllib.request.Request(url + '/api/action', data=b'{"action":"connect","port":"TEST"}',
                                             headers={'Content-Type': 'application/json'})
            with self.assertRaises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(request)
            self.assertEqual(error.exception.code, 403)
            request.add_header('X-Panel-Token', server.token)
            request.add_header('Origin', url)
            with urllib.request.urlopen(request) as response:
                job = json.load(response)['job']
            deadline = time.monotonic() + 2
            while not self.panel.job(job)['done'] and time.monotonic() < deadline:
                time.sleep(0.01)
            with urllib.request.urlopen(url + '/api/status') as response:
                self.assertTrue(json.load(response)['connected'])
            request.add_header('Origin', 'https://example.com')
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(request)
            self.assertEqual(self.fixture.calls, [])
        finally:
            server.shutdown(); server.server_close(); thread.join(timeout=2)


def serve_fixture():
    """专供浏览器验收：显著标注测试数据，设备对象不会打开任何 COM 口。"""
    class FixtureHandler(PanelHandler):
        def do_GET(self):
            if self.path == '/api/ports' and self.valid_host():
                return self.reply(200, [dict(device='TEST-PORT', description='离线界面测试设备', bluetooth=False)])
            super().do_GET()
        def reply(self, code, data, content_type='application/json; charset=utf-8', filename=None):
            if content_type.startswith('text/html'):
                data = data.replace('<body>', '<body><div class="notice error">离线界面测试：所有读数来自测试夹具，没有连接 C 板，不代表机械效果。</div>')
            super().reply(code, data, content_type, filename)
    fixture = FixtureLink()
    panel = PanelController(lambda _: fixture)
    server = PanelServer(('127.0.0.1', 8766), panel)
    server.RequestHandlerClass = FixtureHandler
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        panel.stop(); server.server_close()


if __name__ == '__main__':
    if '--serve-fixture' in sys.argv:
        serve_fixture()
    else:
        unittest.main()

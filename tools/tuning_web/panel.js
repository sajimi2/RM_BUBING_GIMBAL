'use strict';
const $ = id => document.getElementById(id);
const degrees = r => r * 180 / Math.PI;
const token = document.querySelector('meta[name="panel-token"]').content;
let view = null, busy = false, loading = false, initializedPort = null;
let samples = [], lastSequence = 0, closed = false, logCount = 0, observedError = '';
const valid = v => typeof v === 'number' && Number.isFinite(v);
const format = (v, n = 0) => valid(v) ? v.toFixed(n) : '—';
const angle = v => valid(v) ? degrees(v).toFixed(2) + '°' : '—';
function notice(message, error = false) {
  $('notice').textContent = message;
  $('notice').classList.toggle('error', error);
}
function log(message) {
  const row = document.createElement('li');
  row.textContent = new Date().toLocaleTimeString() + ' · ' + message;
  $('log').prepend(row);
  while ($('log').children.length > 15) $('log').lastChild.remove();
  $('log-count').textContent = String(++logCount);
  $('last-action').textContent = message;
}
async function api(path, data) {
  const options = data === undefined ? {} : {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Panel-Token': token}, body: JSON.stringify(data)};
  const response = await fetch(path, {...options, cache: 'no-store', signal: AbortSignal.timeout(10000)});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '本机服务请求失败');
  return result;
}
function fresh() { return view?.connected && view.age_ms !== null && view.age_ms < 1500; }
function gates() {
  const s = view?.state;
  return fresh() && s?.rc_online && s?.automatic && s?.control_enabled;
}
function kiParameter() { return view?.parameters?.find(p => p.id === 3 && p.name === 'pitch.speed.ki'); }
function enableControls() {
  const connected = !!view?.connected, ready = fresh() && !busy;
  $('connect').disabled = busy || connected || !$('port').value || closed;
  $('port').disabled = busy || connected || closed;
  $('refresh').disabled = busy || connected || closed;
  $('disconnect').disabled = busy || !connected || closed;
  for (const id of ['apply','zero','read','current-angle','mark','record-start']) $(id).disabled = !ready;
  $('record-start').disabled ||= !!view?.recording;
  $('record-stop').disabled = busy || !view?.recording;
  $('restore').disabled = !ready || !view?.can_restore;
  for (const id of ['ki-apply','ki-zero','ki-input','ki-down','ki-up']) $(id).disabled = !ready || !kiParameter();
  $('uart').disabled = busy || !gates() || view?.state?.source === 1;
  $('can').disabled = !ready;
  $('send').disabled = busy || !gates() || view?.state?.source !== 1;
  for (const id of ['gain','offset','target','gain-down','gain-up','target-down','target-up']) $(id).disabled = busy || closed;
  $('export').disabled = !view?.recorded_count;
  $('export-snapshots').disabled = !view?.records?.length;
  $('exit').disabled = busy || closed;
}
async function action(actionName, values = {}) {
  if (busy || closed) return false;
  busy = true; enableControls(); notice('正在等待板端确认…');
  try {
    const {job} = await api('/api/action', {action: actionName, ...values});
    const deadline = performance.now() + 12000;
    let result;
    do {
      await new Promise(resolve => setTimeout(resolve, 80));
      result = await api('/api/job/' + job);
    } while (!result.done && performance.now() < deadline);
    if (!result.done) throw new Error('等待超时，不能确定操作是否执行；请检查读回值，不自动重发。');
    if (!result.ok) throw new Error(result.error);
    await refreshStatus(true);
    notice(result.message, result.message.includes('未确认')); log(result.message);
    return true;
  } catch (error) {
    notice(error.message, true); log(error.message);
    return false;
  } finally {
    busy = false; enableControls();
  }
}
function inputNumber(id, name) {
  if (!$(id).value.trim()) throw new Error(name + '不能为空');
  const value = Number($(id).value);
  if (!Number.isFinite(value)) throw new Error(name + '必须是有效数字');
  return value;
}
function tryInput(callback) { try { callback(); } catch (error) { notice(error.message, true); } }
function draftState() {
  const s = view?.state;
  const gain = $('gain').value.trim() ? Number($('gain').value) : NaN;
  const offset = $('offset').value.trim() ? Number($('offset').value) * Math.PI / 180 : NaN;
  const dirty = s && (Math.abs(gain - s.gain) > 0.001 || Math.abs(offset - s.offset_rad) > 0.000001 || !valid(gain) || !valid(offset));
  $('parameter-state').textContent = !s ? '等待板端读回' : dirty ? '输入待应用' : '与板端读回一致';
  $('parameter-state').classList.toggle('pending', !!dirty);
  $('offset-rad').textContent = valid(offset) ? offset.toFixed(4) + ' rad（输入）' : '— rad';
  const ki = $('ki-input').value.trim() ? Number($('ki-input').value) : NaN;
  const kiDirty = s && (!valid(ki) || Math.abs(ki - s.speed_ki) > 0.000001);
  $('ki-state').textContent = !s ? '等待板端读回' : kiDirty ? '输入待应用' : '与板端读回一致';
  $('ki-state').classList.toggle('pending', !!kiDirty);
}
function fillParameters() {
  if (!view?.state) return;
  $('gain').value = valid(view.state.gain) ? String(view.state.gain) : '';
  $('offset').value = valid(view.state.offset_rad) ? degrees(view.state.offset_rad).toFixed(6) : '';
  fillKi();
  draftState();
}
function fillKi() {
  $('ki-input').value = valid(view?.state?.speed_ki) ? String(view.state.speed_ki) : '';
  draftState();
}
function targetDraft() {
  const value = $('target').value.trim() ? Number($('target').value) : NaN;
  $('target-rad').textContent = valid(value) ? (value * Math.PI / 180).toFixed(4) + ' rad' : '— rad';
}
function step(id, amount) {
  if (!$(id).value.trim()) return notice('请先连接设备读回数值，或填写输入框。', true);
  const value = Number($(id).value) + amount;
  $(id).value = id === 'ki-input' ? Math.max(0, Math.min(kiParameter().maximum, Number(value.toFixed(6)))) : Number(value.toFixed(6));
  id === 'target' ? targetDraft() : draftState();
}
function badge(text, tone) {
  const item = document.createElement('span'); item.className = 'badge ' + tone; item.textContent = text; return item;
}
function renderRecords(records) {
  const body = $('records');
  if (body.dataset.count === String(records.length)) return;
  body.dataset.count = String(records.length);
  if (!records.length) return;
  body.replaceChildren();
  records.slice(-20).forEach((row, index) => {
    const tr = document.createElement('tr');
    [records.length - Math.min(records.length, 20) + index + 1, format(row.gain), format(valid(row.offset_rad) ? degrees(row.offset_rad) : null, 2), format(row.speed_ki, 2),
      format(valid(row.target_angle_rad) ? degrees(row.target_angle_rad) : null, 2), format(valid(row.actual_angle_rad) ? degrees(row.actual_angle_rad) : null, 2),
      format(valid(row.angle_error_rad) ? degrees(row.angle_error_rad) : null, 2), row.total_saturated ? '是' : '否'].forEach(text => {
      const td = document.createElement('td'); td.textContent = String(text); tr.append(td);
    }); body.append(tr);
  });
}
function render() {
  const s = view?.state, connected = !!view?.connected, current = fresh();
  $('connection-badge').textContent = connected ? current ? '已连接 · ' + view.port : '已连接 · 数据未刷新' : '未连接设备';
  $('connection-badge').className = 'badge ' + (current ? 'good' : connected ? 'warn' : '');
  $('freshness').textContent = !s ? '尚无实测数据' : connected ? '距最新读数 ' + (view.age_ms / 1000).toFixed(1) + ' 秒' : '已断开 · 保留最后读数';
  $('source-badge').textContent = !s ? '来源未知' : (s.source === 1 ? '串口模拟' : 'CAN 视觉') + (current ? '' : ' · 旧读数');
  $('source-badge').className = 'badge ' + (current && s?.source === 1 ? 'good' : '');
  if (s) {
    for (const [prefix, key] of [['target','target_angle_rad'],['actual','actual_angle_rad'],['error','angle_error_rad']]) {
      $(prefix + '-value').textContent = angle(s[key]);
      $(prefix + '-value-rad').textContent = format(s[key], 4) + ' rad';
    }
    $('gain-current').textContent = format(s.gain);
    $('offset-current').textContent = angle(s.offset_rad);
    $('ki-current').textContent = format(s.speed_ki, 2);
    for (const [id, key, places] of [['p-out','p_out',0],['i-out','i_out',0],['d-out','d_out',1],['pid-out','pid_unclamped_out',0],
      ['gravity-out','gravity_out',0],['total-out','total_out',0],['motor-cmd','motor_cmd',0],['speed','speed_rad_s',4],['ki','speed_ki',0]]) $(id).textContent = format(s[key], places);
    $('flags').replaceChildren(
      badge(s.control_enabled ? '控制已启用' : '控制未启用', s.control_enabled ? 'good' : 'warn'),
      badge(s.rc_online ? '遥控在线' : '遥控掉线', s.rc_online ? 'good' : 'bad'),
      badge(s.automatic ? '自瞄条件满足' : '未进入自瞄', s.automatic ? 'good' : 'warn'),
      ...[['target_limited','目标角限幅'],['pid_saturated','PID 限幅'],['total_saturated','总输出限幅'],
        ['integral_limited','积分限幅'],['integral_blocked','积分被阻止'],['integral_separated','积分分离'],
        ['gravity_limited','重力补偿限幅'],['gravity_config_invalid','重力配置无效']].map(([key, title]) => badge(title + (s[key] ? '：是' : '：否'), s[key] ? 'bad' : ''))
    );
  }
  $('gate-hint').textContent = !connected ? '需先连接设备，再通过遥控器进入自瞄。' : !current ? '读数未刷新，暂不允许发送目标。' : !gates() ? '请打开遥控器、解除无力并进入自瞄。' : s.source === 1 ? '串口模拟中；填入绝对目标，再点击发送。' : '自瞄条件满足，可以开始串口模拟。';
  if (view?.info) {
    const info = view.info;
    $('target-range').textContent = `允许 ${degrees(info.pitch_min_rad).toFixed(2)}° ～ ${degrees(info.pitch_max_rad).toFixed(2)}°。Yaw 保持原目标，不接受开火请求。`;
    $('gain').min = -info.gain_max; $('gain').max = info.gain_max;
    $('offset').min = -degrees(info.offset_max_rad); $('offset').max = degrees(info.offset_max_rad);
    $('target').min = degrees(info.pitch_min_rad); $('target').max = degrees(info.pitch_max_rad);
    $('limits').textContent = `板端限幅：重力补偿 ±${format(info.gravity_max_out)} · 总输出 ±${format(info.total_max_out)}。输出刻度不是 mA。`;
  }
  $('record-status').textContent = `${view?.recording ? '正在记录 · ' : '已记录 '}${view?.recorded_count || 0} 点`;
  const ki = kiParameter();
  $('ki-support').textContent = !connected ? '连接后读取 Ki 的允许范围。' : !ki ? '当前固件不支持实时修改 Ki，请先烧录新版固件。' : `允许 ${ki.minimum}～${ki.maximum}；±100 按钮只修改输入框，点击应用后才下发。`;
  if (ki) { $('ki-input').min = ki.minimum; $('ki-input').max = ki.maximum; }
  $('record-status').classList.toggle('recording', !!view?.recording);
  renderRecords(view?.records || []);
  draftState(); enableControls(); drawChart();
}
async function refreshStatus(force = false) {
  if (closed) return;
  if (loading) {
    if (!force) return;
    while (loading && !closed) await new Promise(resolve => setTimeout(resolve, 30));
    if (closed) return;
  }
  loading = true;
  try {
    const next = await api('/api/status?since=' + lastSequence);
    if (next.connected && initializedPort !== next.port) {
      samples = []; initializedPort = next.port; view = next; fillParameters();
      if (valid(next.state?.actual_angle_rad)) $('target').value = degrees(next.state.actual_angle_rad).toFixed(3);
      targetDraft();
    }
    if (!next.connected) initializedPort = null;
    view = next;
    for (const sample of next.samples) {
      if (sample.seq > lastSequence) { samples.push(sample); lastSequence = sample.seq; }
    }
    samples = samples.slice(-1500);
    if (next.error && next.error !== observedError) { observedError = next.error; notice(next.error, true); log(next.error); }
    if (!next.error) observedError = '';
    render();
  } catch (error) {
    if (view) { view.connected = false; view.age_ms = null; }
    notice('本机服务未响应。不能据此判断云台已停止，请用遥控器退出自瞄。', true);
    render();
  } finally { loading = false; }
}
async function ports() {
  try {
    const previous = $('port').value, values = await api('/api/ports');
    $('port').replaceChildren();
    const empty = document.createElement('option'); empty.value = ''; empty.textContent = '请选择 DAP 的串口'; $('port').append(empty);
    for (const port of values) {
      const option = document.createElement('option'); option.value = port.device;
      option.textContent = port.device + ' · ' + port.description + (port.bluetooth ? '（蓝牙虚拟端口）' : ''); $('port').append(option);
    }
    if (values.some(p => p.device === previous)) $('port').value = previous;
    if (!values.some(p => !p.bluetooth)) notice('当前未发现 USB 串口；请插上 DAP，再点击刷新端口。蓝牙虚拟端口可能仍会显示。');
    enableControls();
  } catch (error) { notice('读取串口失败：' + error.message, true); }
}
function drawChart() {
  const svg = $('chart'); svg.replaceChildren();
  const width = Math.max(300, svg.getBoundingClientRect().width), height = 230, left = 47, right = 13, top = 17, bottom = 27;
  svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
  const latest = samples.length ? samples[samples.length - 1].received_s : 0;
  const points = samples.filter(s => s.received_s >= latest - 60);
  const values = points.flatMap(s => [s.target_angle_rad, s.actual_angle_rad]).filter(valid).map(degrees);
  $('chart-empty').hidden = values.length > 0;
  let min = values.length ? Math.min(...values) : -1, max = values.length ? Math.max(...values) : 1;
  const pad = Math.max(0.3, (max - min) * 0.15); min -= pad; max += pad;
  const start = latest - 60, x = t => left + (t - start) / 60 * (width - left - right);
  const y = value => top + (max - value) / (max - min) * (height - top - bottom);
  function add(tag, attributes, text) {
    const el = document.createElementNS('http://www.w3.org/2000/svg', tag);
    for (const [key, value] of Object.entries(attributes)) el.setAttribute(key, value);
    if (text !== undefined) el.textContent = text; svg.append(el); return el;
  }
  for (let i = 0; i <= 4; i++) {
    const v = min + (max - min) * i / 4;
    add('line', {x1:left, x2:width-right, y1:y(v), y2:y(v), class:'grid'});
    add('text', {x:left-8, y:y(v)+4, 'text-anchor':'end'}, v.toFixed(1));
  }
  for (const t of [-60,-45,-30,-15,0]) add('text', {x:x(latest+t), y:height-6, 'text-anchor':t === 0 ? 'end' : 'middle'}, t === 0 ? '现在' : t + 's');
  for (const [key, cls] of [['target_angle_rad','target'],['actual_angle_rad','actual']]) {
    let path = '', previous = null;
    for (const point of points) {
      if (!valid(point[key])) { previous = null; continue; }
      const move = previous === null || point.received_s - previous > 1.5;
      path += (move ? 'M' : 'L') + x(point.received_s).toFixed(2) + ' ' + y(degrees(point[key])).toFixed(2) + ' ';
      previous = point.received_s;
    }
    add('path', {d:path, class:cls});
  }
}
$('refresh').onclick = ports;
$('port').onchange = enableControls;
$('connect').onclick = () => action('connect', {port:$('port').value});
$('disconnect').onclick = () => action('disconnect');
$('gain-down').onclick = () => step('gain', -100); $('gain-up').onclick = () => step('gain', 100);
$('ki-down').onclick = () => step('ki-input', -100); $('ki-up').onclick = () => step('ki-input', 100);
$('ki-input').oninput = draftState;
$('ki-apply').onclick = () => tryInput(() => {
  const speed_ki = inputNumber('ki-input', 'Ki'), parameter = kiParameter();
  if (!parameter || speed_ki < parameter.minimum || speed_ki > parameter.maximum) throw new Error('Ki 超出板端允许范围');
  action('ki', {speed_ki});
});
$('ki-zero').onclick = async () => { if (await action('ki', {speed_ki:0})) fillKi(); };
$('target-down').onclick = () => step('target', -1); $('target-up').onclick = () => step('target', 1);
$('gain').oninput = draftState; $('offset').oninput = draftState; $('target').oninput = targetDraft;
$('apply').onclick = () => tryInput(() => {
  const gain = inputNumber('gain','Gain'), offset = inputNumber('offset','Offset');
  if (Math.abs(gain) > view.info.gain_max || Math.abs(offset * Math.PI / 180) > view.info.offset_max_rad) throw new Error('输入超出板端参数范围');
  action('parameters', {gain, offset_deg:offset});
});
$('zero').onclick = async () => { if (await action('zero')) fillParameters(); };
$('restore').onclick = async () => { if (await action('restore')) fillParameters(); };
$('read').onclick = async () => { if (await action('read')) fillParameters(); };
$('current-angle').onclick = async () => {
  if (await action('read')) { $('target').value = valid(view.state.actual_angle_rad) ? degrees(view.state.actual_angle_rad).toFixed(3) : ''; targetDraft(); }
};
$('uart').onclick = async () => {
  if (await action('uart')) { $('target').value = degrees(view.state.target_angle_rad).toFixed(3); targetDraft(); }
};
$('can').onclick = () => action('can');
$('send').onclick = () => tryInput(() => {
  const target = inputNumber('target', '目标角度'), r = target * Math.PI / 180;
  if (r < view.info.pitch_min_rad || r > view.info.pitch_max_rad) throw new Error('目标超出板端允许范围');
  action('target', {target_deg:target});
});
$('mark').onclick = () => action('mark');
$('record-start').onclick = () => action('record_start'); $('record-stop').onclick = () => action('record_stop');
$('export').onclick = () => { window.location.href = '/recording.csv'; };
$('export-snapshots').onclick = () => { window.location.href = '/snapshots.csv'; };
$('exit').onclick = async () => {
  if (busy || closed) return;
  busy = true; enableControls();
  try {
    const result = await api('/api/shutdown', {});
    closed = true;
    if (view) view.connected = false;
    notice(result.message + ' 本机调参台已退出，可关闭此页面。', result.message.includes('未确认')); log(result.message); render();
  } catch (error) { notice(error.message, true); }
  finally { busy = false; enableControls(); }
};
new ResizeObserver(drawChart).observe($('chart'));
ports(); refreshStatus();
setInterval(() => refreshStatus(), 300);

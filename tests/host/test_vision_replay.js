// 运行控件实际用的JS和按钮/Timer逻辑；假串口仅收集命令，不连接硬件。
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const assert = require('node:assert/strict');
const folder = path.join(__dirname, '../../tools/vofa/vision_replay');
function load(name) {
    const ctx = {};
    vm.createContext(ctx);
    vm.runInContext(fs.readFileSync(path.join(folder, name), 'utf8').replace('.pragma library', ''), ctx);
    return ctx;
}
const replay = load('replay.js');
const data = load('recording.js');
const recording = data.recording;
const analysis = JSON.parse(fs.readFileSync(path.join(folder, 'recording_analysis.json'), 'utf8'));
assert.equal(recording.yawDeg.length, recording.lastRow - recording.firstRow + 1);
assert.equal(recording.yawDeg.length, analysis.selected_samples);
assert.equal(recording.sourceSha256, analysis.source_sha256);
assert.equal(recording.firstRow, analysis.selected_first_data_row);
assert.equal(recording.lastRow, analysis.selected_last_data_row);
const first = recording.firstRow;
const last = recording.lastRow;
assert.equal(replay.validate(recording, first, last, 1, 20), '');
for (const [a, b, scale, period] of [[first-1,last,1,20], [first,last+1,1,20],
    [last,first,1,20], [first,last,0,20], [first,last,1.1,20], [first,last,1,NaN],
    [first,last,1,4], [first,last,1,101]]) {
    assert.notEqual(replay.validate(recording, a, b, scale, period), '');
}
// 每个原始点、保持、突跳都保留；比例1只平移，比例0.5只缩小。
for (const scale of [1, 0.5]) {
    for (let i = 0; i < recording.yawDeg.length; i++) {
        const expected = replay.wrap(20 + scale * (recording.yawDeg[i] - recording.yawDeg[0]));
        const actual = replay.target(recording, first, last, scale, 20, 20, i * 20);
        assert(Math.abs(actual - expected) < 1e-10);
    }
}
assert.equal(replay.target(recording, first, last, 1, 20, 20, -1), 20);
const finalTarget = replay.target(recording, first, last, 1, 20, 20, 1e9);
assert.equal(finalTarget, replay.wrap(20 + recording.yawDeg.at(-1) - recording.yawDeg[0]));
for (const profile of recording.profiles) {
    assert.equal(replay.target(recording, profile.firstRow, profile.lastRow, 1, 20, 179, 0), 179);
    const range = replay.bounds(recording, profile.firstRow, profile.lastRow, 1);
    assert(range.minimum <= 0 && range.maximum >= 0);
}
assert.equal(replay.wrap(185), -175);
assert.equal(replay.wrap(-185), 175);

const qml = fs.readFileSync(path.join(folder, 'vision_replay.qml'), 'utf8');
assert(!qml.includes('property real scale:')); // 不覆盖Item本身的缩放属性。
function bodyAfter(marker, offset = 0) {
    const markerAt = qml.indexOf(marker, offset);
    assert(markerAt >= 0, marker);
    const start = qml.indexOf('{', markerAt);
    let end = start + 1;
    let depth = 1;
    while (depth) {
        if (qml[end] === '{') depth++;
        if (qml[end] === '}') depth--;
        end++;
    }
    return qml.slice(start + 1, end - 1);
}
let now = 1000;
const sent = [];
const widget = {
    Date: {now: () => now}, Replay: replay, Data: data,
    sys_manager: {connected: true, send_string: text => sent.push(text)},
    ch_menu: {bind_obj: {value: 20}}, profileInput: {currentIndex: 0},
    scaleInput: {text: '1'}, periodInput: {text: '20'},
    lastDataAt: 1000, lastTickAt: 1000, status: '', replaying: false,
    amplitudeScale: 1, sampleMs: 20,
};
widget.root = widget;
vm.createContext(widget);
vm.runInContext('function selectedProfile() {' + bodyAfter('function selectedProfile()') + '}\n' +
    'function start() {' + bodyAfter('function start()') + '}\n' +
    'function stop(message) {' + bodyAfter('function stop(message)') + '}\n' +
    'function tick() {' + bodyAfter('onTriggered:', qml.indexOf('Timer {')) + '}', widget);
assert.equal(sent.length, 0);
widget.start();
assert.equal(widget.replaying, true);
assert.deepEqual(sent, ['yaw_stop:0\n', 'stream:20\n', 'source:1\n', 'yaw:20.0000\n']);
widget.start();
assert.equal(sent.length, 4);
now += 42;
widget.tick();
assert.equal(sent[4], 'yaw:20.0000\n');
now += 42;
widget.tick();
const expectedAt42 = replay.target(recording, first, last, 1, 20, 20, 42);
assert.equal(sent[5], `yaw:${expectedAt42.toFixed(4)}\n`);
widget.stop('manual');
assert.equal(widget.replaying, false);
assert.equal(sent.length, 6);

// 断开串口、无绑定、过期遥测、无效设置均不发接管请求。
function attemptBlocked(change, restore) {
    const length = sent.length;
    change();
    widget.start();
    assert.equal(widget.replaying, false);
    assert.equal(sent.length, length);
    restore();
}
attemptBlocked(() => widget.sys_manager.connected = false, () => widget.sys_manager.connected = true);
attemptBlocked(() => widget.ch_menu.bind_obj = null, () => widget.ch_menu.bind_obj = {value: 20});
attemptBlocked(() => widget.ch_menu.bind_obj.value = NaN, () => widget.ch_menu.bind_obj.value = 20);
attemptBlocked(() => widget.lastDataAt = now - 501, () => widget.lastDataAt = now);
attemptBlocked(() => widget.scaleInput.text = '0', () => widget.scaleInput.text = '1');

// 上传中断和界面卡顿都停止；不会成串补发漏掉的指令。
widget.start();
let count = sent.length;
now += 501;
widget.tick();
assert.equal(widget.replaying, false);
assert.equal(sent.length, count);
widget.lastDataAt = now;
widget.start();
count = sent.length;
now += 201;
widget.lastDataAt = now;
widget.tick();
assert.equal(widget.replaying, false);
assert.equal(sent.length, count);

// 整段按42 ms定时器重放，无自动循环；最后保持终点。
widget.lastDataAt = now;
sent.length = 0;
widget.start();
const duration = recording.yawDeg.length * 20;
while (widget.replaying) {
    now += 42;
    widget.lastDataAt = now;
    widget.tick();
    assert(sent.length < Math.ceil(duration / 42) + 10);
}
assert.equal(sent.at(-1), `yaw:${finalTarget.toFixed(4)}\n`);
assert(now - widget.startedAt >= duration);
assert(now - widget.startedAt < duration + 42);
assert.equal(sent.filter(text => text === 'source:1\n').length, 1);
assert(sent.every(text => /^(yaw_stop:0|stream:20|source:1|yaw:-?\d+\.\d{4})\n$/.test(text)));
console.log(`PASS: all ${recording.yawDeg.length} recorded targets / source manifest / scaling and wrap / actual widget buttons and timer / no autoplay or loop / no burst catch-up / disconnect and stale data / yaw-only commands`);

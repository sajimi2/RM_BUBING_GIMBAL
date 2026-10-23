// 执行控件实际使用的运动公式；不连接VOFA或串口。
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const code = fs.readFileSync(path.join(__dirname, '../../tools/vofa/yaw_sweep/motion.js'), 'utf8')
    .replace('.pragma library', '');
const motion = {};
vm.createContext(motion);
vm.runInContext(code, motion);
assert.equal(motion.validate(3, 0.5, 10), '');
assert.equal(motion.validate(3, 30, 10), '');
for (const parameters of [[0, 0.5, 10], [11, 0.5, 10], [3, 30.1, 10], [3, 0.5, Infinity]]) {
    assert.notEqual(motion.validate(...parameters), '');
}
assert.equal(motion.target(20, 3, 0.5, 0), 20);
assert.equal(motion.target(20, 3, 0.5, 0.5), 23);
assert.equal(motion.target(20, 3, 0.5, 1.5), 17);
assert.equal(motion.target(179, 3, 0.5, 0.5), -178);
assert.equal(motion.target(-179, 3, 0.5, 1.5), 178);
for (let frame = 0; frame < 240; ++frame) {
    const value = motion.target(179, 3, 0.5, frame / 24);
    const displacement = ((value - 179 + 540) % 360) - 180;
    assert(Math.abs(displacement) <= 3.000001);
}
console.log('PASS: widget motion bounds/phase/24 Hz samples/wraparound/invalid settings');

// 提取真实控件中的按钮/Timer函数，假发送口只收集字符串。
const qml = fs.readFileSync(path.join(__dirname, '../../tools/vofa/yaw_sweep/yaw_sweep.qml'), 'utf8');
function bodyAfter(marker, offset = 0) {
    const start = qml.indexOf('{', qml.indexOf(marker, offset));
    assert(start >= 0);
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
    Date: {now: () => now}, Motion: motion,
    sys_manager: {connected: true, send_string: text => sent.push(text)},
    ch_menu: {bind_obj: {value: 20}},
    amplitudeInput: {text: '3'}, frequencyInput: {text: '0.5'}, durationInput: {text: '10'},
    lastDataAt: 1000, status: '', sweeping: false,
};
widget.root = widget;
vm.createContext(widget);
vm.runInContext('function start() {' + bodyAfter('function start()') + '}\n' +
    'function stop(message) {' + bodyAfter('function stop(message)') + '}\n' +
    'function tick() {' + bodyAfter('onTriggered:', qml.indexOf('Timer {')) + '}', widget);
assert.equal(sent.length, 0);
widget.start();
assert.equal(widget.sweeping, true);
assert.equal(widget.center, 20);
assert.deepEqual(sent, ['source:1\n']);
now += 42;
widget.tick();
assert.match(sent[1], /^yaw:20\.\d{4}\n$/);
widget.stop('manual');
assert.equal(widget.sweeping, false);
assert.equal(sent.length, 2); // 停止不发送回中或恢复CAN命令。
widget.sys_manager.connected = false;
widget.start();
assert.equal(widget.sweeping, false);
assert.equal(sent.length, 2);
widget.sys_manager.connected = true;
widget.lastDataAt = now;
widget.start();
now += 501;
widget.tick();
assert.equal(widget.sweeping, false);
assert.equal(sent.length, 3); // 仅新一轮source:1，没有超时后的yaw目标。
widget.lastDataAt = now;
widget.start();
now += 10000;
widget.lastDataAt = now;
widget.tick();
assert.equal(widget.sweeping, false);
assert.equal(sent.length, 4);
console.log('PASS: actual widget start/stop/timer/disconnect/telemetry timeout/duration/no automatic return');
// 高频模式只发配置和保活，不能继续以24 Hz发送假30 Hz目标。
sent.length = 0;
widget.frequencyInput.text = '30';
widget.lastDataAt = now;
widget.start();
assert.equal(widget.highRate, true);
assert.deepEqual(sent, ['source:1\n', 'stream:5\n', 'yaw_wave:3.0000,30.0000,10.000\n']);
now += 100;
widget.lastDataAt = now;
widget.tick();
assert.equal(sent[3], 'yaw_ping:0\n');
widget.stop('manual high rate stop');
assert.deepEqual(sent.slice(-2), ['yaw_stop:0\n', 'stream:20\n']);
assert.equal(widget.sweeping, false);
console.log('PASS: 30 Hz mode/configuration/5ms telemetry/heartbeat/stop/20ms restore');

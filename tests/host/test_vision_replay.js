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
const sim = load('simulation.js');
const dualData = load('dual_recording.js');
const dual = dualData.recording;
const dualReport = JSON.parse(fs.readFileSync(path.join(folder, 'dual_recording_analysis.json'), 'utf8'));
assert.equal(dual.sourceSha256, dualReport.source_sha256);
assert.equal(dual.yawDeg.length, dualReport.source_rows);
assert.equal(dual.pitchDeg.length, dual.yawDeg.length);
assert(dual.trainingRows[1] < dual.holdoutRows[0]);
for (const scale of [1, 0.5]) {
    const trajectory = sim.recorded(dual, 1, dual.lastRow, scale, scale, 20, true);
    assert.equal(trajectory.error, '');
    for (let i = 0; i < dual.yawDeg.length; i++) {
        const target = sim.target(trajectory, 20, -3, i * 20);
        assert(Math.abs(target.yaw - sim.wrap(20 + scale * (dual.yawDeg[i] - dual.yawDeg[0]))) < 1e-10);
        assert(Math.abs(target.pitch - (-3 + scale * (dual.pitchDeg[i] - dual.pitchDeg[0]))) < 1e-10);
    }
}
const crossing = {firstRow: 1, lastRow: 3, yawDeg: [179, -179, -178], pitchDeg: [0, 1, 2]};
assert.deepEqual(Array.from(sim.recorded(crossing, 1, 3, 1, 1, 20, true).yaw), [0, 2, 3]);
const options = {seed: 20261024, seconds: 30, yawScale: 1, pitchScale: 1, sampleMs: 20, yawLimit: 12, pitchLimit: 3};
const synthetic = sim.generate(dual, recording, options);
assert.equal(synthetic.error, '');
assert.equal(JSON.stringify(synthetic), JSON.stringify(sim.generate(dual, recording, {...options})));
assert.notEqual(sim.fingerprint(synthetic), sim.fingerprint(sim.generate(dual, recording, {...options, seed: 7})));
assert(synthetic.chunks.some(chunk => chunk.source === 'latest_pair'));
assert(synthetic.chunks.some(chunk => chunk.source === 'legacy_yaw_new_pitch'));

// 毒化全部留出数据也不能改变生成结果，防止悄悄把验证片段用于训练。
const poisoned = JSON.parse(JSON.stringify(dual));
for (let row = 1; row <= poisoned.lastRow; row++) {
    if (row < dual.trainingRows[0] || row > dual.trainingRows[1]) {
        poisoned.yawDeg[row - 1] = NaN;
        poisoned.pitchDeg[row - 1] = NaN;
    }
}
const poisonedOld = JSON.parse(JSON.stringify(recording));
for (let i = Math.floor(recording.yawDeg.length * 0.75); i < recording.yawDeg.length; i++)
    poisonedOld.yawDeg[i] = NaN;
assert.equal(JSON.stringify(sim.generate(poisoned, poisonedOld, {...options})), JSON.stringify(synthetic));
for (const change of [{seed: -1}, {seed: 0.5}, {seed: 4294967296}, {seconds: 0}, {seconds: 121},
    {yawScale: NaN}, {pitchScale: 2}, {sampleMs: 0}, {yawLimit: 0}, {pitchLimit: Infinity}]) {
    assert.notEqual(sim.generate(dual, recording, {...options, ...change}).error, '');
}
for (let seed = 0; seed < 100; seed++) {
    const t = sim.generate(dual, recording, {...options, seed, seconds: 60});
    assert.equal(t.error, '', `seed ${seed}`);
    const range = sim.bounds(t);
    assert(range.yawMin >= -12 && range.yawMax <= 12);
    assert(range.pitchMin >= -3 && range.pitchMax <= 3);
    assert(t.yaw.every(Number.isFinite) && t.pitch.every(Number.isFinite));
    assert.equal(t.yaw[0], 0);
    assert.equal(t.pitch[0], 0);
    const steps = t.yaw.slice(1).map((x, i) => Math.abs(x - t.yaw[i]));
    assert(Math.max(...steps) < 15); // 接缝没有制造整圈跳变。
}
const pitchOnly = sim.generate(dual, recording, {...options, yawScale: 0});
assert(pitchOnly.yaw.every(x => x === 0));
const yawOnly = sim.generate(dual, recording, {...options, pitchScale: 0});
assert(yawOnly.pitch.every(x => x === 0));
assert(!sim.pitchAllowed(synthetic, -17.7, dual.pitchLimitsDeg));
assert(sim.pitchAllowed(synthetic, 0, dual.pitchLimitsDeg));

function statistics(values) {
    const delta = values.slice(1).map((value, i) => value - values[i]);
    const changed = delta.filter(value => Math.abs(value) > 0.0001);
    const absolute = changed.map(Math.abs).sort((a, b) => a - b);
    const reversals = changed.slice(1).filter((value, i) => value * changed[i] < 0).length;
    return {holdFraction: 1 - changed.length / delta.length,
        stepMedian: absolute[Math.floor(absolute.length * 0.5)],
        stepP95: absolute[Math.floor(absolute.length * 0.95)],
        reversalFraction: reversals / (changed.length - 1)};
}
function jointFraction(trajectory) {
    let either = 0, both = 0;
    for (let i = 1; i < trajectory.yaw.length; i++) {
        const yawChanged = Math.abs(trajectory.yaw[i] - trajectory.yaw[i - 1]) > 0.0001;
        const pitchChanged = Math.abs(trajectory.pitch[i] - trajectory.pitch[i - 1]) > 0.0001;
        if (yawChanged || pitchChanged) either++;
        if (yawChanged && pitchChanged) both++;
    }
    return both / either;
}
const statsByAxis = {yaw: [], pitch: []};
const jointStats = [];
for (let seed = 0; seed < 20; seed++) {
    const t = sim.generate(dual, recording, {...options, seed});
    statsByAxis.yaw.push(statistics(t.yaw));
    statsByAxis.pitch.push(statistics(t.pitch));
    jointStats.push(jointFraction(t));
}
const held = sim.resample(dual, ...dual.holdoutRows, 20, true);
const validation = {model: sim.modelVersion, seeds: '0..19', seconds: 30,
    yawScale: 1, pitchScale: 1, recordPeriodEstimateMs: 20, sendMs: sim.sendMs,
    latestSource: dual.sourceSha256, legacySource: recording.sourceSha256};
validation.bothChangedGivenEither = jointStats.reduce((sum, value) => sum + value, 0) / 20;
assert(validation.bothChangedGivenEither > 0.97);
for (const axis of ['yaw', 'pitch']) {
    const heldStats = statistics(held[axis]);
    const mean = {};
    for (const key of Object.keys(heldStats))
        mean[key] = statsByAxis[axis].reduce((sum, value) => sum + value[key], 0) / 20;
    // 与未参与生成的整段验证数据比较；只检查第一版目标流的大致统计特性。
    assert(mean.stepP95 / heldStats.stepP95 > 0.75 && mean.stepP95 / heldStats.stepP95 < 1.25);
    assert(Math.abs(mean.holdFraction - heldStats.holdFraction) < 0.10);
    assert(Math.abs(mean.reversalFraction - heldStats.reversalFraction) < 0.12);
    validation[axis] = {heldOut: heldStats, syntheticMean: mean};
}
fs.mkdirSync(path.join(folder, '../../../build'), {recursive: true});
fs.writeFileSync(path.join(folder, '../../../build/host-vision-simulation-validation.json'), JSON.stringify(validation, null, 2));

let now = 1000;
const sent = [];
const widgetMath = Object.create(Math);
widgetMath.random = () => 0.25;
const widget = {
    path: 'vision_replay',
    Date: {now: () => now}, Math: widgetMath, Sim: sim, Data: data, Dual: dualData,
    sys_manager: {connected: true, send_string: text => sent.push(text)},
    ch_menu: {bind_obj: {value: 20}, get_ctx: () => ({})},
    pitch_menu: {bind_obj: {value: -3}, get_ctx: () => ({})},
    modeInput: {currentIndex: 0}, profileInput: {currentIndex: 0},
    scaleInput: {text: '1'}, pitchScaleInput: {text: '1'}, periodInput: {text: '20'},
    secondsInput: {text: '5'}, seedInput: {text: '20261024'}, autoSeedInput: {checked: false},
    yawLimitInput: {text: '12'}, pitchLimitInput: {text: '3'},
    lastDataAt: now, lastTickAt: now, status: '', replaying: false,
    amplitudeScale: 1, sampleMs: 20, width: 680, height: 850, minimumWidth: 600, minimumHeight: 560,
    get_ctx: () => ({}), ref: {}, __set_ctx__: (root, ctx) => Object.assign(root, ctx['.']),
};
widget.root = widget;
vm.createContext(widget);
let widgetCode = '';
for (const signature of ['selectedProfile()', 'settings(seed)', 'buildTrajectory(seed)', 'preview()',
    'sendTarget(point)', 'start()', 'stop(message)', 'get_widget_ctx()', 'set_widget_ctx(ctx)']) {
    widgetCode += 'function ' + signature + ' {' + bodyAfter('function ' + signature) + '}\n';
}
widgetCode += 'function tick() {' + bodyAfter('onTriggered:', qml.indexOf('Timer {')) + '}';
vm.runInContext(widgetCode, widget);
assert.equal(sent.length, 0);
widget.start();
assert.equal(widget.replaying, true);
assert.deepEqual(sent, ['yaw_stop:0\n', 'stream:20\n', 'source:1\n', 'aim:-3.0000,20.0000\n']);
widget.start();
assert.equal(sent.length, 4);
now += 42; widget.tick();
assert.equal(sent[4], 'aim:-3.0000,20.0000\n');
now += 42; widget.tick();
const targetAt42 = sim.target(widget.trajectory, 20, -3, 42);
assert.equal(sent[5], `aim:${targetAt42.pitch.toFixed(4)},${targetAt42.yaw.toFixed(4)}\n`);
widget.stop('manual');
function attemptBlocked(change, restore) {
    const length = sent.length;
    change(); widget.start();
    assert.equal(widget.replaying, false);
    assert.equal(sent.length, length);
    restore();
}
attemptBlocked(() => widget.sys_manager.connected = false, () => widget.sys_manager.connected = true);
attemptBlocked(() => widget.ch_menu.bind_obj = null, () => widget.ch_menu.bind_obj = {value: 20});
attemptBlocked(() => widget.pitch_menu.bind_obj = null, () => widget.pitch_menu.bind_obj = {value: -3});
attemptBlocked(() => widget.ch_menu.bind_obj.value = NaN, () => widget.ch_menu.bind_obj.value = 20);
attemptBlocked(() => widget.pitch_menu.bind_obj.value = NaN, () => widget.pitch_menu.bind_obj.value = -3);
attemptBlocked(() => widget.pitch_menu.bind_obj.value = -17.7, () => widget.pitch_menu.bind_obj.value = -3);
attemptBlocked(() => widget.lastDataAt = now - 501, () => widget.lastDataAt = now);
attemptBlocked(() => widget.scaleInput.text = '2', () => widget.scaleInput.text = '1');
widget.start();
let count = sent.length;
now += 501; widget.tick();
assert.equal(widget.replaying, false);
assert.equal(sent.length, count);
widget.lastDataAt = now; widget.start(); count = sent.length;
now += 201; widget.lastDataAt = now; widget.tick();
assert.equal(widget.replaying, false);
assert.equal(sent.length, count);

// 运行三种模式的真实Timer逻辑，检查同帧发送、旧基准兼容和不循环。
for (const mode of [0, 1, 2]) {
    widget.modeInput.currentIndex = mode;
    widget.lastDataAt = now; sent.length = 0; widget.start();
    assert(widget.replaying);
    const duration = widget.trajectory.durationMs;
    while (widget.replaying) {
        now += 42; widget.lastDataAt = now; widget.tick();
        assert(sent.length < Math.ceil(duration / 42) + 10);
    }
    const end = sim.target(widget.trajectory, widget.center, widget.pitchCenter, 1e9);
    let expected = `aim:${end.pitch.toFixed(4)},${end.yaw.toFixed(4)}\n`;
    if (mode === 2) expected = `yaw:${end.yaw.toFixed(4)}\n`;
    assert.equal(sent.at(-1), expected);
    assert.equal(sent.filter(text => text === 'source:1\n').length, 1);
    assert(now - widget.startedAt >= duration && now - widget.startedAt < duration + 42);
    assert(sent.every(text => /^(yaw_stop:0|stream:20|source:1|yaw:-?\d+\.\d{4}|aim:-?\d+\.\d{4},-?\d+\.\d{4})\n$/.test(text)));
}
widget.modeInput.currentIndex = 1;
widget.autoSeedInput.checked = true;
widget.lastDataAt = now; widget.start();
const autoManifest = JSON.parse(widget.lastRunManifest);
assert.equal(autoManifest.seed, 1073741824);
assert.equal(widget.seedInput.text, '1073741824');
const autoTrajectory = JSON.stringify(widget.trajectory);
widget.stop('manual');
widget.autoSeedInput.checked = false;
widget.lastDataAt = now; widget.start();
assert.equal(JSON.stringify(widget.trajectory), autoTrajectory);
assert.equal(JSON.parse(widget.lastRunManifest).fingerprint, autoManifest.fingerprint);
widget.stop('manual');
widgetMath.random = () => 0.5;
widget.autoSeedInput.checked = true;
widget.lastDataAt = now; widget.start();
assert.equal(JSON.parse(widget.lastRunManifest).seed, 2147483648);
assert.notEqual(JSON.parse(widget.lastRunManifest).fingerprint, autoManifest.fingerprint);
widget.stop('manual');
widget.ch_menu.get_ctx = () => ({}); widget.pitch_menu.get_ctx = () => ({});
const saved = widget.get_widget_ctx();
count = sent.length; widget.replaying = true; widget.set_widget_ctx(saved);
assert.equal(widget.replaying, false);
assert.equal(sent.length, count);
console.log(`PASS: legacy ${recording.yawDeg.length} + dual ${dual.yawDeg.length} exact targets / atomic dual widget commands / fixed and automatic seeds / 100 seeds bounded / holdouts excluded and statistics compared / pitch preflight / all three modes / context restore / no autoplay, loop or catch-up / offline only`);

.pragma library

var modelVersion = "dual-block-v1";
var sendMs = 42;

function wrap(angle) {
    return ((angle + 180) % 360 + 360) % 360 - 180;
}

function randomUnit(random) {
    // 显式32位状态，不依赖Math.random；seed=0也能正常生成。
    random.state = (1664525 * random.state + 1013904223) >>> 0;
    return random.state / 4294967296;
}

function randomInteger(random, minimum, maximum) {
    return minimum + Math.floor(randomUnit(random) * (maximum - minimum + 1));
}

function validateScales(yawScale, pitchScale, sampleMs) {
    if (!isFinite(yawScale) || yawScale < 0 || yawScale > 1)
        return "左右摆动幅度请填0～1";
    if (!isFinite(pitchScale) || pitchScale < 0 || pitchScale > 1)
        return "上下摆动幅度请填0～1";
    if (yawScale === 0 && pitchScale === 0)
        return "左右和上下幅度不能同时为0";
    if (!isFinite(sampleMs) || sampleMs < 5 || sampleMs > 100)
        return "原记录每行间隔请填5～100毫秒";
    return "";
}

function validateRows(recording, first, last, withPitch) {
    if (first !== Math.floor(first) || last !== Math.floor(last))
        return "片段行号必须是整数";
    if (first < recording.firstRow || last > recording.lastRow || first >= last)
        return "片段超出记录范围";
    for (var row = first; row <= last; row++) {
        var index = row - recording.firstRow;
        if (!isFinite(recording.yawDeg[index]))
            return "Yaw记录含无效角度";
        if (withPitch) {
            if (!recording.pitchDeg || !isFinite(recording.pitchDeg[index]))
                return "Pitch记录缺失或含无效角度";
        }
    }
    return "";
}

function recorded(recording, first, last, yawScale, pitchScale, sampleMs, withPitch) {
    var error = validateScales(yawScale, pitchScale, sampleMs);
    if (error !== "")
        return {error: error};
    error = validateRows(recording, first, last, withPitch);
    if (error !== "")
        return {error: error};
    var start = first - recording.firstRow;
    var yaw = [0];
    var pitch = [0];
    var accumulatedYaw = 0;
    for (var index = start + 1; index <= last - recording.firstRow; index++) {
        // 先按最短角差展开Yaw，再平移；跨±180°不制造一整圈跳变。
        accumulatedYaw += wrap(recording.yawDeg[index] - recording.yawDeg[index - 1]);
        yaw.push(yawScale * accumulatedYaw);
        var pitchOffset = 0;
        if (withPitch)
            pitchOffset = pitchScale * (recording.pitchDeg[index] - recording.pitchDeg[start]);
        pitch.push(pitchOffset);
    }
    return {error: "", yaw: yaw, pitch: pitch, sampleMs: sampleMs,
            durationMs: yaw.length * sampleMs, kind: "recorded", firstRow: first, lastRow: last};
}

function resample(recording, first, last, sampleMs, withPitch) {
    var original = recorded(recording, first, last, 1, 1, sampleMs, withPitch);
    if (original.error !== "")
        return original;
    var yaw = [];
    var pitch = [];
    var pitchEvents = [0];
    for (var elapsed = 0; elapsed < original.durationMs; elapsed += sendMs) {
        var index = Math.floor(elapsed / sampleMs);
        yaw.push(original.yaw[index]);
        pitch.push(original.pitch[index]);
        // 旧记录只有Yaw时，借用本次Pitch的连续变化序列，共用旧Yaw的更新时刻。
        if (Math.abs(original.pitch[index] - pitchEvents[pitchEvents.length - 1]) > 0.0001)
            pitchEvents.push(original.pitch[index]);
    }
    return {error: "", yaw: yaw, pitch: pitch, pitchEvents: pitchEvents};
}

function validateOptions(options) {
    var error = validateScales(options.yawScale, options.pitchScale, options.sampleMs);
    if (error !== "")
        return error;
    if (!isFinite(options.seed) || options.seed !== Math.floor(options.seed))
        return "轨迹编号请填0～4294967295之间的整数";
    if (options.seed < 0 || options.seed > 4294967295)
        return "轨迹编号请填0～4294967295之间的整数";
    if (!isFinite(options.seconds) || options.seconds < 1 || options.seconds > 120)
        return "模拟时长应在1～120秒之间";
    if (!isFinite(options.yawLimit) || options.yawLimit <= 0 || options.yawLimit > 90)
        return "左右偏离范围请填大于0、不超过90度的数";
    if (!isFinite(options.pitchLimit) || options.pitchLimit <= 0 || options.pitchLimit > 20)
        return "上下偏离范围请填大于0、不超过20度的数";
    return "";
}

function fitBlock(block, options, previousYaw, previousPitch) {
    var directions = [[1, 1], [-1, 1], [1, -1], [-1, -1]];
    for (var direction = 0; direction < directions.length; direction++) {
        var yaw = [];
        var pitch = [];
        var fits = true;
        for (var frame = 0; frame < block.yaw.length; frame++) {
            var nextYaw = previousYaw + directions[direction][0] * block.yaw[frame];
            var nextPitch = previousPitch + directions[direction][1] * block.pitch[frame];
            // 整块可镜像回到中心；不硬裁角度制造平台或边界尖峰。
            if (Math.abs(nextYaw) > options.yawLimit || Math.abs(nextPitch) > options.pitchLimit) {
                fits = false;
                break;
            }
            yaw.push(nextYaw);
            pitch.push(nextPitch);
        }
        if (fits) {
            block.yaw = yaw;
            block.pitch = pitch;
            block.yawGain *= directions[direction][0];
            block.pitchGain *= directions[direction][1];
            return block;
        }
    }
    return null;
}

function candidate(random, paired, legacy, options, previousYaw, previousPitch, remaining, maximumLength) {
    // 七成抽取本次成对片段；三成使用上次Yaw形状配本次Pitch片段。
    // 上次没有Pitch原列，后者不冒充旧记录的真实双轴配对。
    var source = paired;
    var sourceName = "latest_pair";
    if (randomUnit(random) < 0.3) {
        source = legacy;
        sourceName = "legacy_yaw_new_pitch";
    }
    var minimumLength = Math.min(12, maximumLength);
    var length = Math.min(remaining, randomInteger(random, minimumLength, maximumLength));
    var timeScale = 0.9 + 0.2 * randomUnit(random);
    var span = Math.ceil((length - 1) * timeScale);
    if (source.yaw.length <= span + 2 || paired.pitch.length <= span + 2)
        return null;
    var sourceStart = randomInteger(random, 1, source.yaw.length - span - 1);
    var pitchStart = sourceStart;
    if (sourceName === "legacy_yaw_new_pitch") {
        if (paired.pitchEvents.length <= length + 2)
            return null;
        pitchStart = randomInteger(random, 1, paired.pitchEvents.length - length - 1);
    }
    var yawGain = options.yawScale * (0.9 + 0.2 * randomUnit(random));
    var pitchGain = options.pitchScale * (0.85 + 0.3 * randomUnit(random));
    var yaw = [];
    var pitch = [];
    var previousLegacyYaw = source.yaw[sourceStart - 1];
    var legacyPitchIndex = pitchStart;
    var legacyPitchValue = paired.pitchEvents[pitchStart - 1];
    for (var frame = 0; frame < length; frame++) {
        var advance = Math.floor(frame * timeScale);
        // 从片段前一个点对齐，接缝第一步也使用真实增量，不额外插入停顿。
        var nextYaw = yawGain * (source.yaw[sourceStart + advance] - source.yaw[sourceStart - 1]);
        var nextPitch;
        if (sourceName === "legacy_yaw_new_pitch") {
            var legacyYaw = source.yaw[sourceStart + advance];
            // Yaw保持时Pitch也保持，避免独立抽样破坏实录的双轴共同更新节奏。
            if (Math.abs(legacyYaw - previousLegacyYaw) > 0.0001) {
                legacyPitchValue = paired.pitchEvents[legacyPitchIndex];
                legacyPitchIndex++;
            }
            nextPitch = pitchGain * (legacyPitchValue - paired.pitchEvents[pitchStart - 1]);
            previousLegacyYaw = legacyYaw;
        }
        else {
            nextPitch = pitchGain * (paired.pitch[pitchStart + advance] - paired.pitch[pitchStart - 1]);
        }
        yaw.push(nextYaw);
        pitch.push(nextPitch);
    }
    var block = {yaw: yaw, pitch: pitch, source: sourceName, sourceStart: sourceStart,
                 pitchStart: pitchStart, timeScale: timeScale, yawGain: yawGain, pitchGain: pitchGain};
    return fitBlock(block, options, previousYaw, previousPitch);
}

function generate(recording, legacyRecording, options) {
    var error = validateOptions(options);
    if (error !== "")
        return {error: error};
    var paired = resample(recording, recording.trainingRows[0], recording.trainingRows[1], options.sampleMs, true);
    if (paired.error !== "")
        return paired;
    // 旧Yaw最后四分之一也留作验证，生成器只读取前四分之三。
    var legacyLast = legacyRecording.firstRow + Math.floor(legacyRecording.yawDeg.length * 0.75) - 1;
    var legacy = resample(legacyRecording, legacyRecording.firstRow, legacyLast, legacyRecording.sampleMs, false);
    if (legacy.error !== "")
        return legacy;
    if (paired.yaw.length < 45 || legacy.yaw.length < 45)
        return {error: "可用记录太短，至少需要约2秒的连续记录"};
    var random = {state: options.seed >>> 0};
    var trajectory = {error: "", yaw: [0], pitch: [0], sampleMs: sendMs,
                      durationMs: options.seconds * 1000, seed: options.seed, kind: "synthetic",
                      modelVersion: modelVersion, options: options, chunks: [],
                      sourceHashes: [recording.sourceSha256, legacyRecording.sourceSha256]};
    var count = Math.ceil(trajectory.durationMs / sendMs);
    while (trajectory.yaw.length < count) {
        var block = null;
        var previous = trajectory.yaw.length - 1;
        // 有限次重抽，范围过窄时明确拒绝，不在界面线程中无限循环。
        for (var attempt = 0; attempt < 64; attempt++) {
            var maximumLength = 36;
            // 接近范围边缘时，后半轮尝试短片段，给真实反向增量留出拼接机会。
            if (attempt >= 32)
                maximumLength = 6;
            block = candidate(random, paired, legacy, options, trajectory.yaw[previous],
                              trajectory.pitch[previous], count - trajectory.yaw.length, maximumLength);
            if (block !== null)
                break;
        }
        if (block === null)
            return {error: "摆动放不进设定范围，请减小摆动幅度，或在高级设置中放宽偏离范围"};
        trajectory.chunks.push({firstFrame: trajectory.yaw.length, source: block.source,
                               sourceStart: block.sourceStart, pitchStart: block.pitchStart,
                               frames: block.yaw.length, yawGain: block.yawGain,
                               pitchGain: block.pitchGain, timeScale: block.timeScale});
        for (var frame = 0; frame < block.yaw.length; frame++) {
            trajectory.yaw.push(block.yaw[frame]);
            trajectory.pitch.push(block.pitch[frame]);
        }
    }
    return trajectory;
}

function bounds(trajectory) {
    var result = {yawMin: 0, yawMax: 0, pitchMin: 0, pitchMax: 0};
    for (var index = 0; index < trajectory.yaw.length; index++) {
        result.yawMin = Math.min(result.yawMin, trajectory.yaw[index]);
        result.yawMax = Math.max(result.yawMax, trajectory.yaw[index]);
        result.pitchMin = Math.min(result.pitchMin, trajectory.pitch[index]);
        result.pitchMax = Math.max(result.pitchMax, trajectory.pitch[index]);
    }
    return result;
}

function target(trajectory, yawCenter, pitchCenter, elapsedMs) {
    var index = Math.floor(Math.max(0, elapsedMs) / trajectory.sampleMs);
    index = Math.min(index, trajectory.yaw.length - 1);
    return {yaw: wrap(yawCenter + trajectory.yaw[index]), pitch: pitchCenter + trajectory.pitch[index]};
}

function pitchAllowed(trajectory, center, limits) {
    var range = bounds(trajectory);
    if (center + range.pitchMin < limits[0])
        return false;
    if (center + range.pitchMax > limits[1])
        return false;
    return true;
}

function fingerprint(trajectory) {
    // 标识实际生成的角度序列；相同seed/设置/模型应得到相同标识。
    var hash = 2166136261;
    for (var index = 0; index < trajectory.yaw.length; index++) {
        var values = [Math.round(trajectory.yaw[index] * 1000000), Math.round(trajectory.pitch[index] * 1000000)];
        for (var axis = 0; axis < values.length; axis++) {
            hash = hash ^ values[axis];
            // 简单滚动标识，不作安全哈希；乘积小于2^53，可精确保留整数。
            hash = (1664525 * (hash >>> 0) + 1013904223) >>> 0;
        }
    }
    return ("00000000" + hash.toString(16)).slice(-8);
}

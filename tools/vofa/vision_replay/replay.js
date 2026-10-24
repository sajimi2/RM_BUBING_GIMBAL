.pragma library

function wrap(angle) {
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

function validate(recording, first, last, scale, sampleMs) {
    if (!isFinite(scale) || scale <= 0 || scale > 1)
        return "幅度比例应大于0且不超过1";
    if (!isFinite(sampleMs) || sampleMs < 5 || sampleMs > 100)
        return "记录采样周期请输入5～100 ms";
    if (first !== Math.floor(first) || last !== Math.floor(last))
        return "数据行必须为整数";
    if (first < recording.firstRow || last > recording.lastRow || first >= last)
        return "所选数据行超出已提取的视觉段";
    for (var row = first; row <= last; row++) {
        if (!isFinite(recording.yawDeg[row - recording.firstRow]))
            return "记录含无效目标，停止重放";
    }
    return "";
}

// 只平移起点和等比例缩小角度变化；不平滑、不插值，保留实测跳变形状。
function target(recording, first, last, scale, sampleMs, center, elapsedMs) {
    var index = Math.floor(elapsedMs / sampleMs);
    if (index < 0)
        index = 0;
    var count = last - first + 1;
    if (index >= count)
        index = count - 1;
    var startIndex = first - recording.firstRow;
    var difference = recording.yawDeg[startIndex + index] - recording.yawDeg[startIndex];
    return wrap(center + scale * difference);
}

function bounds(recording, first, last, scale) {
    var origin = recording.yawDeg[first - recording.firstRow];
    var minimum = 0;
    var maximum = 0;
    for (var row = first; row <= last; row++) {
        var offset = scale * (recording.yawDeg[row - recording.firstRow] - origin);
        minimum = Math.min(minimum, offset);
        maximum = Math.max(maximum, offset);
    }
    return {minimum: minimum, maximum: maximum};
}

.pragma library

// 下发频率约24 Hz；frequency表示左右摆动次数/秒，两者相互独立。
function validate(amplitude, frequency, duration) {
    if (!isFinite(amplitude) || amplitude <= 0 || amplitude > 10)
        return "幅度请输入0～10°之间的数（不含0）";
    if (!isFinite(frequency) || frequency < 0.1 || frequency > 2)
        return "摆动频率请输入0.1～2 Hz";
    if (!isFinite(duration) || duration < 1 || duration > 60)
        return "时长请输入1～60秒";
    return "";
}

function target(center, amplitude, frequency, elapsed) {
    var angle = center + amplitude * Math.sin(2 * Math.PI * frequency * elapsed);
    // 对当前中心附近的小幅摆动，最多跨过一次整圈边界。
    if (angle > 180)
        angle -= 360;
    if (angle < -180)
        angle += 360;
    return angle;
}

import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0
import "recording.js" as Data
import "dual_recording.js" as Dual
import "simulation.js" as Sim

ResizableRectangle {
    id: root
    property string path: "vision_replay"
    property real amplitudeScale: 0.5
    property real pitchScale: 0.5
    property real sampleMs: Dual.recording.sampleMs
    property int replayMode: 0
    property double seedValue: 20261024
    property bool randomSeed: false
    property real seconds: 30
    property real yawLimit: 12
    property real pitchLimit: 3
    property bool replaying: false
    property int activeMode: 0
    property var trajectory: null
    property real center: 0
    property real pitchCenter: 0
    property double startedAt: 0
    property double lastTickAt: 0
    property double lastDataAt: 0
    property string lastRunManifest: ""
    property string status: "尚未开始；遥控切到自瞄后点开始。"
    property Item ref: Loader {
        active: false
        sourceComponent: Component {
            Item {
                property var ref_ch_menu: ch_menu
                property var ref_pitch_menu: pitch_menu
            }
        }
    }

    width: g_settings.applyHScale(680)
    height: g_settings.applyVScale(850)
    minimumWidth: g_settings.applyHScale(600)
    minimumHeight: g_settings.applyVScale(560)
    color: "#f5f8fb"
    border.color: "#b9c8d7"
    radius: 6

    MyMenu {
        id: menu
        MyMenu {
            title: "Yaw实际角（I2）"
            ChMenu { id: ch_menu }
        }
        MyMenu {
            title: "Pitch实际角（I0）"
            ChMenu { id: pitch_menu }
        }
        DeleteMenuItem { target: root }
    }
    Connections {
        target: root.mouse
        onClicked: {
            if (mouse.button === Qt.RightButton)
                menu.popup();
        }
    }
    Connections {
        target: sys_manager
        onNeed_update: root.lastDataAt = Date.now()
        onConnectedChanged: {
            if (!sys_manager.connected)
                root.stop("串口已断开，停止发送");
        }
    }

    function selectedProfile() {
        if (modeInput.currentIndex === 2)
            return Data.recording.profiles[profileInput.currentIndex];
        return Dual.recording.profiles[profileInput.currentIndex];
    }

    function settings(seed) {
        return {seed: seed, seconds: Number(secondsInput.text),
                yawScale: Number(scaleInput.text), pitchScale: Number(pitchScaleInput.text),
                sampleMs: Number(periodInput.text), yawLimit: Number(yawLimitInput.text),
                pitchLimit: Number(pitchLimitInput.text)};
    }

    function buildTrajectory(seed) {
        var options = settings(seed);
        if (modeInput.currentIndex === 1)
            return Sim.generate(Dual.recording, Data.recording, options);
        var profile = selectedProfile();
        if (!profile)
            return {error: "请选择片段"};
        if (modeInput.currentIndex === 2)
            return Sim.recorded(Data.recording, profile.firstRow, profile.lastRow,
                                options.yawScale, 0, options.sampleMs, false);
        return Sim.recorded(Dual.recording, profile.firstRow, profile.lastRow,
                            options.yawScale, options.pitchScale, options.sampleMs, true);
    }

    function preview() {
        // 预览不消耗随机数；只有点击开始时才生成一整轮模拟轨迹。
        if (modeInput.currentIndex === 1) {
            var previewSeed = Number(seedInput.text);
            if (autoSeedInput.checked)
                previewSeed = 0;
            var error = Sim.validateOptions(settings(previewSeed));
            if (error !== "")
                return error;
            return "这一轮运行 " + secondsInput.text + " 秒。";
        }
        var value = buildTrajectory(Number(seedInput.text));
        if (value.error !== "")
            return value.error;
        return "这段记录约 " + (value.durationMs / 1000).toFixed(1) + " 秒，播放一次后停止。";
    }

    function sendTarget(point) {
        // 旧Yaw基准仍只改Yaw；双轴模式只用一条命令提交一对角度。
        if (activeMode === 2) {
            sys_manager.send_string("yaw:" + point.yaw.toFixed(4) + "\n");
            return;
        }
        sys_manager.send_string("aim:" + point.pitch.toFixed(4) + "," + point.yaw.toFixed(4) + "\n");
    }

    function start() {
        // 重复点击不重新接管，也不重置当前轮次。
        if (replaying)
            return;
        if (!sys_manager.connected) {
            status = "先连接串口";
            return;
        }
        if (!ch_menu.bind_obj) {
            status = "右键 → Yaw实际角 → 绑定I2";
            return;
        }
        var yaw = Number(ch_menu.bind_obj.value);
        if (!isFinite(yaw) || Math.abs(yaw) > 180) {
            status = "Yaw绑定无效；I2请用增益1、偏置0";
            return;
        }
        var pitch = 0;
        if (modeInput.currentIndex !== 2) {
            if (!pitch_menu.bind_obj) {
                status = "右键 → Pitch实际角 → 绑定I0";
                return;
            }
            pitch = Number(pitch_menu.bind_obj.value);
            if (!isFinite(pitch)) {
                status = "Pitch绑定无效；I0请用增益1、偏置0";
                return;
            }
        }
        if (Date.now() - lastDataAt > 500) {
            status = "没有收到新数据，请检查串口和上传开关。";
            return;
        }
        var seed = Number(seedInput.text);
        if (modeInput.currentIndex === 1 && autoSeedInput.checked)
            seed = Math.floor(Math.random() * 4294967296);
        var next = buildTrajectory(seed);
        if (next.error !== "") {
            status = next.error;
            return;
        }
        if (modeInput.currentIndex !== 2) {
            if (!Sim.pitchAllowed(next, pitch, Dual.recording.pitchLimitsDeg)) {
                status = "上下摆动会超过限位，请调整起始角度或减小上下幅度。尚未开始。";
                return;
            }
        }
        trajectory = next;
        center = yaw;
        pitchCenter = pitch;
        activeMode = modeInput.currentIndex;
        seedValue = seed;
        seedInput.text = seed.toString();
        amplitudeScale = Number(scaleInput.text);
        pitchScale = Number(pitchScaleInput.text);
        sampleMs = Number(periodInput.text);
        seconds = Number(secondsInput.text);
        yawLimit = Number(yawLimitInput.text);
        pitchLimit = Number(pitchLimitInput.text);
        randomSeed = autoSeedInput.checked;
        lastRunManifest = JSON.stringify({mode: activeMode, seed: seed, model: Sim.modelVersion,
            fingerprint: Sim.fingerprint(next), yawCenter: center, pitchCenter: pitchCenter, durationMs: next.durationMs,
            options: settings(seed), firstRow: next.firstRow, lastRow: next.lastRow,
            latestSource: Dual.recording.sourceSha256, legacySource: Data.recording.sourceSha256});
        // 先完成全部校验，再一次性请求接管；不改PID/前馈，不携带开火标志。
        sys_manager.send_string("yaw_stop:0\n");
        sys_manager.send_string("stream:20\n");
        sys_manager.send_string("source:1\n");
        sendTarget(Sim.target(trajectory, center, pitchCenter, 0));
        startedAt = Date.now() + 42;
        lastTickAt = Date.now();
        replaying = true;
        status = "正在开始，请观察目标角曲线。";
    }

    function stop(message) {
        replaying = false;
        status = message;
        // 停止不回中、不恢复CAN，不产生额外目标跳变。
    }

    Timer {
        interval: 42
        repeat: true
        running: root.replaying
        onTriggered: {
            var now = Date.now();
            if (!sys_manager.connected || now - root.lastDataAt > 500) {
                root.stop("通信中断，停止发送");
                return;
            }
            // 卡顿或时钟回退后停止，不成串补发过期指令。
            if (now - root.lastTickAt > 200 || now < root.lastTickAt) {
                root.stop("电脑发送卡顿，已停止，请重新开始。");
                return;
            }
            root.lastTickAt = now;
            var elapsed = now - root.startedAt;
            if (elapsed < 0)
                return;
            var point = Sim.target(root.trajectory, root.center, root.pitchCenter, elapsed);
            root.sendTarget(point);
            if (elapsed >= root.trajectory.durationMs) {
                root.stop("本轮完成，保持最后目标；退出自瞄请用遥控器");
                return;
            }
            root.status = "正在运行：" + (elapsed / 1000).toFixed(1) + " / "
                    + (root.trajectory.durationMs / 1000).toFixed(1) + " 秒";
        }
    }

    ScrollView {
        id: scroller
        anchors.fill: parent
        anchors.margins: 14
        clip: true
        ColumnLayout {
            width: scroller.availableWidth
            spacing: 10
            Label { text: "视觉指令模拟"; font.bold: true; font.pixelSize: g_settings.applyVScale(26) }
            Label {
                text: "让云台按记录运动，或生成一条新轨迹。只瞄准，不发射。"
                font.pixelSize: g_settings.applyVScale(18)
                wrapMode: Text.Wrap
                Layout.fillWidth: true
            }
            GridLayout {
                columns: 2
                Layout.fillWidth: true
                enabled: !root.replaying
                Label { text: "怎么模拟"; font.pixelSize: g_settings.applyVScale(20) }
                ComboBox {
                    id: modeInput
                    model: ["重播这次记录（左右＋上下）", "随机生成一轮（左右＋上下）", "重播上次记录（只动左右）"]
                    currentIndex: root.replayMode
                    onCurrentIndexChanged: {
                        root.replayMode = currentIndex;
                    }
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(20)
                }
                Label { text: "播放哪一段"; visible: modeInput.currentIndex !== 1; font.pixelSize: g_settings.applyVScale(20) }
                ComboBox {
                    id: profileInput
                    // 显示名称与数据分开，顺序仍对应原记录片段，不改采样范围。
                    Component.onCompleted: updateModel()
                    function updateModel() {
                        // 上次记录的三个选项，仍按原有顺序对应数据。
                        if (modeInput.currentIndex === 2) {
                            model = ["完整记录", "开头一段", "变化密集的一段"];
                        } else {
                            model = ["常用片段", "另一段（用于复查参数）", "完整记录（含开头大跳变）"];
                        }
                    }
                    Connections {
                        target: modeInput
                        onCurrentIndexChanged: {
                            profileInput.updateModel();
                            profileInput.currentIndex = 0;
                        }
                    }
                    visible: modeInput.currentIndex !== 1
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(20)
                }
                Label { text: "左右摆动幅度"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: scaleInput; text: root.amplitudeScale.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "上下摆动幅度"; visible: modeInput.currentIndex !== 2; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: pitchScaleInput; text: root.pitchScale.toString(); visible: modeInput.currentIndex !== 2; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "运行多久（秒）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: secondsInput; text: root.seconds.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
            }
            Label {
                text: "幅度填 0～1：1 = 原幅度，0.5 = 一半，0 = 这一轴不动。"
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                font.pixelSize: g_settings.applyVScale(17)
                color: "#4c6075"
            }
            CheckBox {
                id: autoSeedInput
                text: "每次开始都换一条新轨迹"
                checked: root.randomSeed
                visible: modeInput.currentIndex === 1
                enabled: !root.replaying
                font.pixelSize: g_settings.applyVScale(20)
            }
            GridLayout {
                columns: 2
                Layout.fillWidth: true
                visible: modeInput.currentIndex === 1 && !autoSeedInput.checked
                enabled: !root.replaying
                Label { text: "轨迹编号"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: seedInput; text: root.seedValue.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
            }
            Label {
                text: "想重复刚才那一轮？取消勾选，保留轨迹编号和其他设置，再点开始。"
                visible: modeInput.currentIndex === 1
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                font.pixelSize: g_settings.applyVScale(17)
                color: "#4c6075"
            }
            Label { text: root.preview(); wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            RowLayout {
                Button { text: "开始"; enabled: !root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.start() }
                Button { text: "停止"; enabled: root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.stop("已停止，保持最后目标。退出自瞄请用遥控器。") }
            }
            Label { text: root.status; wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            Label {
                text: "从当前角度开始。停止后保持最后目标，退出自瞄请用遥控器。"
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                font.pixelSize: g_settings.applyVScale(17)
                color: "#4c6075"
            }
            ToolButton {
                id: advancedToggle
                text: {
                    // 展开后给出收起提示，避免用户不知道这一行可以点击。
                    if (checked)
                        return "▾ 首次使用 / 高级设置";
                    return "▸ 首次使用 / 高级设置";
                }
                checkable: true
                checked: false
                font.pixelSize: g_settings.applyVScale(18)
            }
            ColumnLayout {
                visible: advancedToggle.checked
                Layout.fillWidth: true
                spacing: 10
                Label {
                    text: "首次使用：在控件背景右键，把Yaw实际角绑定到I2、Pitch实际角绑定到I0。两个通道都用增益1、偏置0。"
                    wrapMode: Text.Wrap
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(17)
                }
                GridLayout {
                    columns: 2
                    Layout.fillWidth: true
                    enabled: !root.replaying
                    Label { text: "原记录每行间隔（毫秒）"; font.pixelSize: g_settings.applyVScale(18) }
                    TextField { id: periodInput; text: root.sampleMs.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(20) }
                    Label { text: "左右最多偏离起点（±度）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(18) }
                    TextField { id: yawLimitInput; text: root.yawLimit.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(20) }
                    Label { text: "上下最多偏离起点（±度）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(18) }
                    TextField { id: pitchLimitInput; text: root.pitchLimit.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(20) }
                }
                Label {
                    text: "原记录没有时间戳，目前按每行20毫秒估计；不确定就保持原值。这不是向板子发送指令的间隔。"
                    wrapMode: Text.Wrap
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(17)
                    color: "#4c6075"
                }
                ToolButton {
                    id: detailsToggle
                    text: {
                        // 详细数据只在需要复制时显示，保持常用操作区域简洁。
                        if (checked)
                            return "▾ 本轮详细记录（可复制）";
                        return "▸ 查看本轮详细记录（可复制）";
                    }
                    checkable: true
                    checked: false
                    font.pixelSize: g_settings.applyVScale(17)
                }
                TextArea {
                    text: root.lastRunManifest
                    visible: detailsToggle.checked
                    readOnly: true
                    selectByMouse: true
                    wrapMode: TextEdit.Wrap
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(15)
                }
            }
        }
    }

    function get_widget_ctx() {
        return {path: path, ctx: {
            ".": {ctx: get_ctx(), amplitudeScale: Number(scaleInput.text), pitchScale: Number(pitchScaleInput.text),
                  sampleMs: Number(periodInput.text), replayMode: modeInput.currentIndex, seedValue: Number(seedInput.text),
                  randomSeed: autoSeedInput.checked, seconds: Number(secondsInput.text), yawLimit: Number(yawLimitInput.text),
                  pitchLimit: Number(pitchLimitInput.text), lastRunManifest: lastRunManifest},
            ch_menu: {ctx: ch_menu.get_ctx()}, pitch_menu: {ctx: pitch_menu.get_ctx()}
        }};
    }
    function set_widget_ctx(ctx) {
        replaying = false;
        __set_ctx__(root, ctx.ctx, ref);
        if (width < minimumWidth)
            width = minimumWidth;
        if (height < minimumHeight)
            height = minimumHeight;
    }
}

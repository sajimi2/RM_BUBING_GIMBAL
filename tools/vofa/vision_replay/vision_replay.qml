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
    property string status: "右键分别绑定Yaw实际角I2、Pitch实际角I0，遥控器进入自瞄"
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
            return "模拟" + secondsInput.text + "秒；两轴同帧发送，开始后显示实际种子和轨迹标识";
        }
        var value = buildTrajectory(Number(seedInput.text));
        if (value.error !== "")
            return value.error;
        var range = Sim.bounds(value);
        return "约" + (value.durationMs / 1000).toFixed(2) + "秒；Yaw偏移" + range.yawMin.toFixed(2)
                + "～" + range.yawMax.toFixed(2) + "°；Pitch偏移" + range.pitchMin.toFixed(2)
                + "～" + range.pitchMax.toFixed(2) + "°";
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
            status = "没有新遥测，停止启动";
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
                status = "预计Pitch越过限位，请调整初始姿态或减小Pitch比例；未发送";
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
        status = "已请求启动；查看I1/I3目标角。轨迹标识 " + Sim.fingerprint(next);
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
                root.stop("发送调度中断，请重新开始本轮");
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
            root.status = "发送 " + (elapsed / 1000).toFixed(1) + " / "
                    + (root.trajectory.durationMs / 1000).toFixed(1) + "秒；Yaw " + point.yaw.toFixed(2)
                    + "°，Pitch " + point.pitch.toFixed(2) + "°";
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
            Label { text: "视觉双轴重播 / 种子模拟"; font.bold: true; font.pixelSize: g_settings.applyVScale(26) }
            Label {
                text: "只瞄准，不发射；双轴模式需要支持aim命令的新固件"
                font.pixelSize: g_settings.applyVScale(18)
                wrapMode: Text.Wrap
                Layout.fillWidth: true
            }
            GridLayout {
                columns: 2
                Layout.fillWidth: true
                enabled: !root.replaying
                Label { text: "模式"; font.pixelSize: g_settings.applyVScale(20) }
                ComboBox {
                    id: modeInput
                    model: ["本次双轴实录", "双轴种子模拟", "上次Yaw基准（保持Pitch）"]
                    currentIndex: root.replayMode
                    onCurrentIndexChanged: {
                        root.replayMode = currentIndex;
                    }
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(20)
                }
                Label { text: "重播片段"; visible: modeInput.currentIndex !== 1; font.pixelSize: g_settings.applyVScale(20) }
                ComboBox {
                    id: profileInput
                    model: Dual.recording.profiles
                    // 绑定表达式只选择数据，不运行发送或生成逻辑。
                    Component.onCompleted: updateModel()
                    function updateModel() {
                        if (modeInput.currentIndex === 2)
                            model = Data.recording.profiles;
                        else
                            model = Dual.recording.profiles;
                    }
                    Connections {
                        target: modeInput
                        onCurrentIndexChanged: {
                            profileInput.updateModel();
                            profileInput.currentIndex = 0;
                        }
                    }
                    textRole: "name"
                    visible: modeInput.currentIndex !== 1
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(20)
                }
                Label { text: "Yaw幅度比例（0～1）"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: scaleInput; text: root.amplitudeScale.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "Pitch幅度比例（0～1）"; visible: modeInput.currentIndex !== 2; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: pitchScaleInput; text: root.pitchScale.toString(); visible: modeInput.currentIndex !== 2; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "记录周期估计（ms）"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: periodInput; text: root.sampleMs.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "模拟时长（秒）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: secondsInput; text: root.seconds.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "固定种子"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: seedInput; text: root.seedValue.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "每轮自动换种子"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                CheckBox { id: autoSeedInput; checked: root.randomSeed; visible: modeInput.currentIndex === 1 }
                Label { text: "Yaw最大偏移（±°）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: yawLimitInput; text: root.yawLimit.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
                Label { text: "Pitch最大偏移（±°）"; visible: modeInput.currentIndex === 1; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: pitchLimitInput; text: root.pitchLimit.toString(); visible: modeInput.currentIndex === 1; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(22) }
            }
            Label { text: root.preview(); wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            RowLayout {
                Button { text: "开始发送"; enabled: !root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.start() }
                Button { text: "停止发送"; enabled: root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.stop("已停止，保持最后目标") }
            }
            Label { text: root.status; wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            Label { text: "本轮配置（可选中复制，含种子和轨迹标识）"; font.pixelSize: g_settings.applyVScale(16) }
            TextArea { text: root.lastRunManifest; readOnly: true; selectByMouse: true; wrapMode: TextEdit.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(15) }
            Label {
                text: "约24Hz发送，起点平移到当前实际角。20ms是记录周期估计，没有硬件时间戳。固定种子对照参数，换种子检验泛化；独立留出段不参与生成。"
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                font.pixelSize: g_settings.applyVScale(17)
                color: "#4c6075"
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

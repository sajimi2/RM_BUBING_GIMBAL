import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0
import "recording.js" as Data
import "replay.js" as Replay

ResizableRectangle {
    id: root
    property string path: "vision_replay"
    property real amplitudeScale: 1
    property real sampleMs: Data.recording.sampleMs
    property int firstRow: Data.recording.firstRow
    property int lastRow: Data.recording.lastRow
    property bool replaying: false
    property real center: 0
    property double startedAt: 0
    property double lastTickAt: 0
    property double lastDataAt: 0
    property string status: "右键绑定I2（Yaw实际角），遥控器进入自瞄"
    property Item ref: Loader {
        active: false
        sourceComponent: Component {
            Item { property var ref_ch_menu: ch_menu }
        }
    }

    width: g_settings.applyHScale(620)
    height: g_settings.applyVScale(590)
    minimumWidth: g_settings.applyHScale(570)
    minimumHeight: g_settings.applyVScale(550)
    color: "#f5f8fb"
    border.color: "#b9c8d7"
    radius: 6

    MyMenu {
        id: menu
        ChMenu { id: ch_menu }
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
                root.stop("串口已断开，停止重放");
        }
    }

    function selectedProfile() {
        return Data.recording.profiles[profileInput.currentIndex];
    }

    function preview() {
        var profile = selectedProfile();
        if (!profile)
            return "";
        var value = Number(scaleInput.text);
        var period = Number(periodInput.text);
        var error = Replay.validate(Data.recording, profile.firstRow, profile.lastRow, value, period);
        if (error !== "")
            return error;
        var range = Replay.bounds(Data.recording, profile.firstRow, profile.lastRow, value);
        var seconds = (profile.lastRow - profile.firstRow + 1) * period / 1000;
        return "预计" + seconds.toFixed(2) + "秒，起点相对变化" + range.minimum.toFixed(2) + "～" + range.maximum.toFixed(2) + "°";
    }

    function start() {
        // 正在重放时再次点击不能重复接管或重新计时。
        if (replaying)
            return;
        if (!sys_manager.connected) {
            status = "先连接串口";
            return;
        }
        if (!ch_menu.bind_obj) {
            status = "右键 → 绑定数据 → I2（Yaw实际角）";
            return;
        }
        var angle = Number(ch_menu.bind_obj.value);
        if (!isFinite(angle) || Math.abs(angle) > 180) {
            status = "绑定角度无效；请确认I2增益1、偏置0";
            return;
        }
        if (Date.now() - lastDataAt > 500) {
            status = "没有新遥测，停止启动";
            return;
        }
        var profile = selectedProfile();
        var value = Number(scaleInput.text);
        var period = Number(periodInput.text);
        var error = Replay.validate(Data.recording, profile.firstRow, profile.lastRow, value, period);
        if (error !== "") {
            status = error;
            return;
        }
        amplitudeScale = value;
        sampleMs = period;
        firstRow = profile.firstRow;
        lastRow = profile.lastRow;
        center = angle;
        // 只在点击开始时接管一次。沿用现有只瞄准命令，不发送开火标志。
        sys_manager.send_string("yaw_stop:0\n");
        sys_manager.send_string("stream:20\n");
        sys_manager.send_string("source:1\n");
        sys_manager.send_string("yaw:" + center.toFixed(4) + "\n");
        // 给初始化命令留一个发送周期，之后才开始记录轨迹的时间。
        startedAt = Date.now() + 42;
        lastTickAt = Date.now();
        replaying = true;
        status = "已请求接管，检查I3目标角；只重放Yaw，Pitch保持原目标";
    }

    function stop(message) {
        replaying = false;
        status = message;
        // 停止不回中、不恢复CAN，不引起额外目标跳变。
    }

    Timer {
        interval: 42
        repeat: true
        running: root.replaying
        onTriggered: {
            var now = Date.now();
            if (!sys_manager.connected || now - root.lastDataAt > 500) {
                root.stop("通信中断，停止重放");
                return;
            }
            // 界面卡顿后停止，不成串补发已经错过的历史指令。
            if (now - root.lastTickAt > 200 || now < root.lastTickAt) {
                root.stop("发送调度中断，请重新开始本轮");
                return;
            }
            root.lastTickAt = now;
            var elapsed = now - root.startedAt;
            if (elapsed < 0)
                return;
            var duration = (root.lastRow - root.firstRow + 1) * root.sampleMs;
            var angle = Replay.target(Data.recording, root.firstRow, root.lastRow,
                                      root.amplitudeScale, root.sampleMs, root.center, elapsed);
            sys_manager.send_string("yaw:" + angle.toFixed(4) + "\n");
            if (elapsed >= duration) {
                root.stop("本轮完成，保持最后目标；退出自瞄请用遥控器");
                return;
            }
            root.status = "重放 " + (elapsed / 1000).toFixed(1) + " / " + (duration / 1000).toFixed(1) + "秒，目标" + angle.toFixed(2) + "°";
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
            Label { text: "实测视觉Yaw重放"; font.bold: true; font.pixelSize: g_settings.applyVScale(26) }
            Label {
                text: Data.recording.baselineLabel
                font.pixelSize: g_settings.applyVScale(18)
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                color: "#187456"
            }
            Label {
                text: "保留实测台阶、突跳和保持，约24 Hz发送；只瞄准，不发射"
                font.pixelSize: g_settings.applyVScale(18)
                wrapMode: Text.Wrap
                Layout.fillWidth: true
            }
            GridLayout {
                columns: 2
                Layout.fillWidth: true
                enabled: !root.replaying
                Label { text: "重放片段"; font.pixelSize: g_settings.applyVScale(20) }
                ComboBox {
                    id: profileInput
                    model: Data.recording.profiles
                    textRole: "name"
                    Layout.fillWidth: true
                    font.pixelSize: g_settings.applyVScale(20)
                }
                Label { text: "幅度比例（0～1）"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: scaleInput; text: root.amplitudeScale.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(24) }
                Label { text: "记录采样周期（ms）"; font.pixelSize: g_settings.applyVScale(20) }
                TextField { id: periodInput; text: root.sampleMs.toString(); Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(24) }
            }
            Label { text: root.preview(); wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            RowLayout {
                Button { text: "开始重放"; enabled: !root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.start() }
                Button { text: "停止发送"; enabled: root.replaying; font.pixelSize: g_settings.applyVScale(20); onClicked: root.stop("已停止，保持最后目标") }
            }
            Label { text: root.status; wrapMode: Text.Wrap; Layout.fillWidth: true; font.pixelSize: g_settings.applyVScale(18) }
            Label {
                text: "起点平移到当前实际角。20 ms来自数据交叉估计；原CSV无时间戳。两轮对比保持片段、比例、周期一致。"
                wrapMode: Text.Wrap
                Layout.fillWidth: true
                font.pixelSize: g_settings.applyVScale(18)
                color: "#4c6075"
            }
        }
    }

    function get_widget_ctx() {
        return {path: path, ctx: {
            ".": {ctx: get_ctx(), amplitudeScale: amplitudeScale, sampleMs: sampleMs},
            ch_menu: {ctx: ch_menu.get_ctx()}
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

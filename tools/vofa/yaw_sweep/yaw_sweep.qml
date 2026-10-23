import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import MyModules 1.0
import "motion.js" as Motion

ResizableRectangle {
    id: root
    property string path: "yaw_sweep"
    property real amplitude: 3
    property real frequency: 0.5
    property real duration: 10
    property bool sweeping: false
    property real center: 0
    property double startedAt: 0
    property double lastDataAt: 0
    property string status: "先右键绑定 I2（Yaw实际角），遥控拨到自瞄"
    property Item ref: Loader {
        active: false
        sourceComponent: Component {
            Item { property var ref_ch_menu: ch_menu }
        }
    }

    width: g_settings.applyHScale(430)
    height: g_settings.applyVScale(480)
    minimumWidth: g_settings.applyHScale(410)
    minimumHeight: g_settings.applyVScale(460)
    color: "#f5f8fb"
    border.color: "#b9c8d7"
    border.width: 1
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
                root.stop("串口已断开，停止发送");
        }
    }

    function start() {
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
            status = "绑定角度无效；请确认绑定I2，增益1、偏置0";
            return;
        }
        if (Date.now() - lastDataAt > 500) {
            status = "未收到新遥测，请确认固件正在上传";
            return;
        }
        var a = Number(amplitudeInput.text);
        var f = Number(frequencyInput.text);
        var d = Number(durationInput.text);
        var error = Motion.validate(a, f, d);
        if (error !== "") {
            status = error;
            return;
        }
        amplitude = a;
        frequency = f;
        duration = d;
        center = angle;
        startedAt = Date.now();
        // 只在点击开始时切一次来源；遥控退出后不自动重新接管。
        sys_manager.send_string("source:1\n");
        sweeping = true;
        status = "已发送接管请求；以I3目标角读回为准";
    }

    function stop(message) {
        sweeping = false;
        status = message;
        // 不自动回中心或恢复CAN，避免停止按钮引起额外转动。
    }

    Timer {
        interval: 42
        repeat: true
        running: root.sweeping
        onTriggered: {
            var now = Date.now();
            if (!sys_manager.connected || now - root.lastDataAt > 500) {
                root.stop("通信中断，停止发送");
                return;
            }
            var elapsed = (now - root.startedAt) / 1000;
            if (elapsed < 0 || elapsed >= root.duration) {
                root.stop("本轮结束，保持最后目标；退出自瞄请用遥控器");
                return;
            }
            var angle = Motion.target(root.center, root.amplitude, root.frequency, elapsed);
            sys_manager.send_string("yaw:" + angle.toFixed(4) + "\n");
            root.status = "发送目标 " + angle.toFixed(2) + "° · " + elapsed.toFixed(1) + " s";
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 10
        Label { text: "Yaw 小幅连续摆动"; font.bold: true; font.pixelSize: 18 }
        Label { text: "约24次/秒下发 · 以点击开始时的实际Yaw为中心"; color: "#4c6075" }
        GridLayout {
            columns: 2
            Layout.fillWidth: true
            enabled: !root.sweeping
            Label { text: "左右幅度（±°）" }
            TextField { id: amplitudeInput; text: root.amplitude.toString(); Layout.fillWidth: true }
            Label { text: "摆动频率（Hz）" }
            TextField { id: frequencyInput; text: root.frequency.toString(); Layout.fillWidth: true }
            Label { text: "本轮时长（秒）" }
            TextField { id: durationInput; text: root.duration.toString(); Layout.fillWidth: true }
        }
        RowLayout {
            Button { text: "开始摆动"; enabled: !root.sweeping; onClicked: root.start() }
            Button { text: "停止发送"; enabled: root.sweeping; onClicked: root.stop("已停止发送，保持最后目标") }
        }
        Label { text: root.status; wrapMode: Text.Wrap; Layout.fillWidth: true }
        Label { text: "需烧录支持yaw命令的固件；不会自动开始，结束不自动回中。"; wrapMode: Text.Wrap; Layout.fillWidth: true; color: "#4c6075" }
        Item { Layout.fillHeight: true }
    }

    function get_widget_ctx() {
        return {path: path, ctx: {
            ".": {ctx: get_ctx(), amplitude: amplitude, frequency: frequency, duration: duration},
            ch_menu: {ctx: ch_menu.get_ctx()}
        }};
    }
    function set_widget_ctx(ctx) {
        sweeping = false;
        __set_ctx__(root, ctx.ctx, ref);
    }
}

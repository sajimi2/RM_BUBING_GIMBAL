/* 编译期检查跨编译器迁移最容易改变的 CAN 帧与遥控器结构体布局。 */
#include <stddef.h>
#include "NX_communicate.h"
#include "remote_control.h"

_Static_assert(sizeof(CAN_Send_to_MCU) == 8, "NX CAN frame must remain 8 bytes");
_Static_assert(offsetof(CAN_Send_to_MCU, yaw) == 1, "NX yaw offset changed");
_Static_assert(offsetof(CAN_Send_to_MCU, pitch) == 3, "NX pitch offset changed");
_Static_assert(offsetof(CAN_Send_to_MCU, fire) == 5, "NX fire offset changed");
_Static_assert(offsetof(CAN_Send_to_MCU, rev) == 6, "NX rev offset changed");
_Static_assert(sizeof(CAN_send_to_NX) == 8, "Quaternion CAN frame must remain 8 bytes");
_Static_assert(sizeof(RC_ctrl_t) == 22, "Remote control packed layout changed");
_Static_assert(offsetof(RC_ctrl_t, mouse) == 12, "Remote mouse offset changed");
_Static_assert(offsetof(RC_ctrl_t, key) == 20, "Remote key offset changed");

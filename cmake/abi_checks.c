/* 迁移编译器后，通信结构体仍须保持约定的字节布局。 */
#include <stddef.h>
#include "NX_communicate.h"
#include "remote_control.h"

_Static_assert(sizeof(CAN_Send_to_MCU) == 8, "Vision CAN payload must remain 8 bytes");
_Static_assert(offsetof(CAN_Send_to_MCU, yaw) == 1, "Vision yaw offset changed");
_Static_assert(offsetof(CAN_Send_to_MCU, pitch) == 3, "Vision pitch offset changed");
_Static_assert(sizeof(RC_ctrl_t) == 22, "Remote control packed layout changed");

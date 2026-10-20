#ifndef TUNING_SERVICE_H
#define TUNING_SERVICE_H

#include "tuning_protocol.h"

/* 调用者负责串口收发；服务层只执行完整且校验正确的请求。 */
void tuning_service_request(const tuning_message_t *request, tuning_message_t *reply);
void tuning_service_state(tuning_message_t *message);
uint16_t tuning_service_period_ms(void);

#endif

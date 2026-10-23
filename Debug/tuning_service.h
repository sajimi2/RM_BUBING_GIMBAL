#ifndef TUNING_SERVICE_H
#define TUNING_SERVICE_H

#include "tuning_protocol.h"

void tuning_service_request(const tuning_message_t *request);
/* 固定八通道遥测，不向JustFloat数据流混入文本或额外回执通道。 */
uint16_t tuning_service_frame(uint8_t output[TUNING_FRAME_SIZE]);
uint16_t tuning_service_period_ms(void);

#endif

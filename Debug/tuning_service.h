#ifndef TUNING_SERVICE_H
#define TUNING_SERVICE_H

#include "tuning_protocol.h"

void tuning_service_request(const tuning_message_t *request);
/* 同一帧包含实际参数、执行结果和状态，不向 JustFloat 数据流混入文本。 */
uint16_t tuning_service_frame(uint8_t output[TUNING_FRAME_SIZE]);
uint16_t tuning_service_period_ms(void);

#endif

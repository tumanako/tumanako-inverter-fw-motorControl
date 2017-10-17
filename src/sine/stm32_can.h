#ifndef STM32_CAN_H_INCLUDED
#define STM32_CAN_H_INCLUDED
#include "params.h"

#ifdef __cplusplus
extern "C"
{
#endif

void can_setup(void);
void can_send(uint32_t canId, uint8_t* data, uint32_t len);
void can_sendall();
int can_addsend(Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain);

#ifdef __cplusplus
}
#endif

#endif

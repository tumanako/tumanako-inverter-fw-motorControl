#ifndef STM32_CAN_H_INCLUDED
#define STM32_CAN_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

void can_setup(void);
void can_send(uint32_t canId, uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif

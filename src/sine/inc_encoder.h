#ifndef INC_ENCODER_H_INCLUDED
#define INC_ENCODER_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void enc_init(void);
uint16_t enc_get_angle();
uint32_t enc_get_speed();
void enc_set_filter_const(uint8_t flt);
void enc_set_imp_per_rev(uint16_t imp);

#ifdef __cplusplus
}
#endif

#endif // INC_ENCODER_H_INCLUDED

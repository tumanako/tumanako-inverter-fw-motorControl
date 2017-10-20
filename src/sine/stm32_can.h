#ifndef STM32_CAN_H_INCLUDED
#define STM32_CAN_H_INCLUDED
#include "params.h"

#define CAN_ERR_INVALID_ID -1
#define CAN_ERR_INVALID_OFS -2
#define CAN_ERR_INVALID_LEN -3
#define CAN_ERR_MAXMAP -4
#define CAN_ERR_MAXITEMS -5

namespace Can
{
   void Clear(void);
   void Setup(void);
   void Send(uint32_t canId, uint8_t* data, uint32_t len);
   void SendAll();
   void Save();
   int AddSend(Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain);
   int AddRecv(Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain);
}


#endif

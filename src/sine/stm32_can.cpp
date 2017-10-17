#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include "stm32_can.h"

#define MAX_ENTRIES 10
#define MAX_ITEMS 8
#define CAN_ERR_INVALID_ID -1
#define CAN_ERR_INVALID_OFS -2
#define CAN_ERR_INVALID_LEN -3
#define CAN_ERR_MAXMAP -4
#define CAN_ERR_MAXITEMS -5
#define SDO_WRITE 0x40
#define SDO_READ 0x22
#define SDO_ABORT 0x80
#define SDO_WRITE_REPLY 0x23
#define SDO_READ_REPLY 0x43
#define SDO_ERR_INVIDX 0x06020000
#define SDO_ERR_RANGE 0x06090030

#if ((MAX_ITEMS * 4 + 4) * MAX_ENTRIES + 4) > 512
#error CANMAP will not fit in one flash page
#endif

typedef struct
{
   union
   {
      Param::PARAM_NUM mapParam:8;
      uint8_t id;
   };

   s32fp gain:8;
   uint8_t offsetBits;
   uint8_t numBits;
} CANPOS;

typedef struct
{
   uint16_t canId;
   uint8_t currentItem;
   CANPOS items[MAX_ITEMS];
} CANIDMAP;

typedef struct
{
   uint8_t currentItem;
   CANIDMAP items[MAX_ENTRIES];
} CANMAP;

typedef struct
{
   uint8_t cmd;
   uint16_t index;
   uint8_t subIndex;
   uint32_t data;
} __attribute__((packed)) CAN_SDO;

static CANMAP canSendMap;
static CANMAP canRecvMap;
CANIDMAP *can_find(CANMAP *canMap, int canId);

static int can_add(CANMAP *canMap, Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain)
{
   if (canId > 0x7ff) return CAN_ERR_INVALID_ID;
   if (offset > 63) return CAN_ERR_INVALID_OFS;
   if (length > 32) return CAN_ERR_INVALID_LEN;

   CANIDMAP *existingMap = can_find(canMap, canId);

   if (0 == existingMap)
   {
      if (canMap->currentItem == MAX_ENTRIES)
         return CAN_ERR_MAXMAP;

      existingMap = &canMap->items[canMap->currentItem];
      existingMap->canId = canId;
      canMap->currentItem++;
   }

   if (existingMap->currentItem == MAX_ENTRIES)
      return CAN_ERR_MAXITEMS;

   CANPOS &currentItem = existingMap->items[existingMap->currentItem];

   currentItem.mapParam = param;
   currentItem.gain = gain;
   currentItem.offsetBits = offset;
   currentItem.numBits = length;
   existingMap->currentItem++;

   return canMap->currentItem;
}

int can_addsend(Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain)
{
   return can_add(&canSendMap, param, canId, offset, length, gain);
}

int can_addrecv(Param::PARAM_NUM param, int canId, int offset, int length, s32fp gain)
{
   return can_add(&canRecvMap, param, canId, offset, length, gain);
}

CANIDMAP *can_find(CANMAP *canMap, int canId)
{
   for (int i = 0; i < canMap->currentItem; i++)
   {
      if (canMap->items[i].canId == canId)
         return &canMap->items[i];
   }
   return 0;
}

//http://www.byteme.org.uk/canopenparent/canopen/sdo-service-data-objects-canopen/
static void ProcessSDO(uint8_t* data)
{
   CAN_SDO *sdo = (CAN_SDO*)data;
   if (sdo->index == 0x2000 && sdo->subIndex < Param::PARAM_LAST)
   {
      if (sdo->cmd == SDO_WRITE)
      {
         if (Param::Set((Param::PARAM_NUM)sdo->subIndex, sdo->data) == 0)
         {
            sdo->cmd = SDO_WRITE_REPLY;
         }
         else
         {
            sdo->cmd = SDO_ABORT;
            sdo->data = SDO_ERR_RANGE;
         }
      }
      else if (sdo->cmd == SDO_READ)
      {
         sdo->data = Param::Get((Param::PARAM_NUM)sdo->subIndex);
         sdo->cmd = SDO_READ_REPLY;
      }
   }
   else
   {
      sdo->cmd = SDO_ABORT;
      sdo->data = SDO_ERR_INVIDX;
   }
   can_send(0x581, data, 8);
}

void can_save()
{
/*   ReplaceIndexById(canSendMap);
   ReplaceIndexById(canRecvMap);

   CRC_CR |= CRC_CR_RESET;

   for (idx = 0; idx < PARAM_WORDS; idx++)
   {
      CRC_DR
      uint32_t* pData = ((uint32_t*)&parmPage) + idx;
      flash_program_word(PARAM_ADDRESS + idx * sizeof(uint32_t), *pData);
   }*/
}

void can_sendall()
{
   for (CANIDMAP *curMap = canSendMap.items; curMap->currentItem > 0; curMap++)
   {
      uint32_t data[2] = { 0 }; //Had an issue with uint64_t, otherwise would have used that

      for (int j = 0; j < curMap->currentItem; j++)
      {
         CANPOS &curItem = curMap->items[j];
         s32fp val = FP_MUL(Param::Get(curItem.mapParam), curItem.gain);

         val &= ((1 << curItem.numBits) - 1);

         if (curItem.offsetBits > 31)
         {
            data[1] |= val << (curItem.offsetBits - 32);
         }
         else
         {
            data[0] |= val << curItem.offsetBits;
         }
      }
      can_send(curMap->canId, (uint8_t*)&data, 8);
   }
}

void can_setup(void)
{
   //printf("%d\n", sizeof(canSendMap));
   for (int i = 0; i < MAX_ENTRIES; i++)
   {
      canSendMap.items[i].currentItem = 0;
      canRecvMap.items[i].currentItem = 0;
   }

	// Enable peripheral clocks.
	rcc_periph_clock_enable(RCC_AFIO);
	//rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_CAN1);

	AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTA;

	// Configure CAN pin: RX (input pull-up).
	gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
	gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

	// Configure CAN pin: TX.-
	gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);

	// NVIC setup.
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	//lowest priority
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4);

	// Reset CAN
	can_reset(CAN1);

	// CAN cell init.
	 // Setting the bitrate to 250KBit. APB1 = 36MHz,
	 // prescaler = 9 -> 4MHz time quanta frequency.
	 // 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
	 // 16time quanto per bit period, therefor 4MHz/16 = 250kHz
	 //
	can_init(CAN1,
		     false,          // TTCM: Time triggered comm mode?
		     true,           // ABOM: Automatic bus-off management?
		     false,          // AWUM: Automatic wakeup mode?
		     true,           // NART: No automatic retransmission?
		     false,          // RFLM: Receive FIFO locked mode?
		     false,          // TXFP: Transmit FIFO priority?
		     CAN_BTR_SJW_1TQ,
		     CAN_BTR_TS1_9TQ,
		     CAN_BTR_TS2_6TQ,
		     9,				// BRP+1: Baud rate prescaler
		     false,
		     false);

	// CAN filter 0 init.
	can_filter_id_mask_32bit_init(CAN1,
				0,     // Filter ID
				0,     // CAN ID
				0,     // CAN ID mask
				0,     // FIFO assignment (here: FIFO0)
				true); // Enable the filter.

	// Enable CAN RX interrupt.
	can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void can_send(uint32_t canId, uint8_t* data, uint32_t len)
{
	can_transmit(CAN1,
				 canId,     /* (EX/ST)ID: CAN ID */
				 false, /* IDE: CAN ID extended? */
				 false, /* RTR: Request transmit? */
				 len,     /* DLC: Data length */
				 data);
}

void usb_lp_can_rx0_isr(void)
{
	uint32_t id, fmi;
	bool ext, rtr;
	uint8_t length;
	uint32_t data[2];

	can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, (uint8_t*)&data);
	can_fifo_release(CAN1, 0);

	if (id == 0x601) //SDO request, nodeid=1
   {
      ProcessSDO((uint8_t*)&data);
   }

	/*CANIDMAP *recvMap = can_find(&canRecvMap, id);

   for (int i = 0; i < recvMap->currentItem; i++)
   {
      CANPOS &curItem = recvMap->items[i];
      int val = (data >> curItem.offsetBits) & ((1 << curItem.numBits) - 1);
      Param::SetDig(curItem.mapParam, val);
   }

      for (int j = 0; j < curMap->currentItem; j++)
      {
         CANPOS &curItem = curMap->items[j];
         int val
         s32fp val = FP_MUL(Param::Get(curItem.mapParam), curItem.gain);

         val &= ((1 << curItem.numBits) - 1);

         if (curItem.offsetBits > 31)
         {
            data[1] |= val << (curItem.offsetBits - 32);
         }
         else
         {
            data[0] |= val << curItem.offsetBits;
         }
      }*/
}

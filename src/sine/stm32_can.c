#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "stm32_can.h"

/*struct can_tx_msg {
	uint32_t std_id;
	uint32_t ext_id;
	uint8_t ide;
	uint8_t rtr;
	uint8_t dlc;
	uint8_t data[8];
};

struct can_rx_msg {
	uint32_t std_id;
	uint32_t ext_id;
	uint8_t ide;
	uint8_t rtr;
	uint8_t dlc;
	uint8_t data[8];
	uint8_t fmi;
};

struct can_tx_msg can_tx_msg;
struct can_rx_msg can_rx_msg;*/

void can_setup(void)
{
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
	//nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	//lowest priority
	//nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4);

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
	//can_enable_irq(CAN1, CAN_IER_FMPIE0);
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
	uint8_t length, data[8];

	can_receive(CAN1, 0, false, &id, &ext, &rtr, &fmi, &length, data);

	// process the CAN message here ...

	can_fifo_release(CAN1, 0);
}

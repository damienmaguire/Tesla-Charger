
//#define STM32F1

#define PARM_SJW	CAN_BTR_SJW_1TQ
#define PARM_TS1	CAN_BTR_TS1_4TQ
#define PARM_TS2	CAN_BTR_TS2_3TQ
#define PARM_BRP	9		// 500 kbps


#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/can.h>
#include <stdio.h>
#include "stm32scheduler.h"
#include "Chg_Can.h"
#include "usart.h"

static Stm32Scheduler* scheduler;



static void Ms1Task(void)
{

}

static void Ms10Task(void)
{

}

static void Ms100Task(void)
{
gpio_toggle(GPIOC, GPIO13);
iwdg_reset();
SendBMW();

}





extern "C" void tim4_isr(void)
{
   scheduler->Run();
}



int main(void) {



    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_TIM4);//enable timer 4 for scheduler
     rcc_periph_clock_enable(RCC_DMA1);  //ADC, Encoder and UART receive
    rcc_periph_clock_enable(RCC_AFIO); //CAN
    rcc_periph_clock_enable(RCC_CAN1); //CAN
    nvic_enable_irq(NVIC_TIM4_IRQ);//enable timer 4 irq


    rcc_periph_clock_enable(RCC_GPIOA);//enable all io ports
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

	        gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,GPIO_CAN_PB_TX);
	        gpio_set_mode(GPIOB,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIO_CAN_PB_RX);

	        gpio_primary_remap(                             // Map CAN1 to use PB8/PB9
	                AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,      // Optional
	                AFIO_MAPR_CAN1_REMAP_PORTB);            // CAN_RX=PB8, CAN_TX=PB9
	 can_reset(CAN1);

	         can_init(
                CAN1,
                false,                                  // ttcm=off
                false,                                  // auto bus off management
                true,                                   // Automatic wakeup mode.
                false,                                   // No automatic retransmission.
                false,                                 // Receive FIFO locked mode
                false,                                  // Transmit FIFO priority (msg id)
                PARM_SJW,	                        // Resynchronization time quanta jump width (0..3)
                PARM_TS1,				// segment 1 time quanta width
                PARM_TS2,	                        // Time segment 2 time quanta width
		PARM_BRP,				// Baud rate prescaler for 33.333 kbs
		false,					// Loopback
		false);					// Silent

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
   nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4); //lowest priority

   nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
   nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf << 4); //lowest priority





    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    usart_setup();

    Stm32Scheduler s(TIM4); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms1Task, 100);
   s.AddTask(Ms10Task, 1000);
   s.AddTask(Ms100Task, 10000);

	/* Blink the LED (PC12) on the board with every transmitted byte. */
	while (1) {

	}
	return 0;
}



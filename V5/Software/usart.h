#include <libopencm3/stm32/usart.h>
#define TERM_USART         USART3
#define TERM_USART_TXPIN   GPIO_USART3_TX
#define TERM_USART_TXPORT  GPIOB
#define TERM_USART_DMARX   DMA_CHANNEL3
#define TERM_USART_DMATX   DMA_CHANNEL2 //this means we can not use it on rev1 hardware (TIM3_CH3)
#define TERM_USART_DR      USART3_DR
#define TERM_BUFSIZE       128

extern "C"
{


void usart_setup(void);

}

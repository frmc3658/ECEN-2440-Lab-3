#include "msp.h"
#include "gpio.c"


#define IRQn38
#define IQRn35


/**
 * main.c
 */


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // MAKE ATOMIC
    __disable_irq();               //Disable IRQ while configuring
    __NVIC_DisableIRQ(PORT4_IRQn);
    __NVIC_DisableIRQ(PORT1_IRQn);


    init_game(P4, P1, P2, P5, EXT_SW, OB_SW, LLEDS, RLEDS, BIT1, PORT4_IRQn, PORT1_IRQn);


    while(1)
    {

    }
}




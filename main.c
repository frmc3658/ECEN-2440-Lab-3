#include "msp.h"
#include "gpio.h"


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



    init_game();


    while(1)
    {

    }
}




#include "msp.h"
#include <stdio.h>
#include <stdint.h>


#define LEDS        0xC7 //(1100 0111)
#define EXT_SW      0x30 //(0011 0000)
#define SW          0x12 //(0001 0010)
#define HIGH        BIT0 //(0000 0001)
#define LOW         0x00

#define SELECT      (P4->IFG & BIT5)
#define sel_TRUE    BIT5
#define RESET       (P4->IFG & BIT4)
#define r_TRUE      BIT4
#define STOP        (P1->IFG & BIT4)
#define sto_TRUE    BIT4
#define START       (P1->IFG & BIT1)
#define sta_TRUE    BIT1

#define IRQn38
#define IQRn35

volatile int counter = 0;
volatile uint8_t Lout_mem = 0x00;
volatile uint8_t Rout_mem = 0x00;


/**
 * main.c
 */
void sm_delay(void)
{
    int i = 0;
    for(i = 10000; i > 0; i--);
}

void input_config(void)
{
    P4->DIR = ~EXT_SW; //Set P4.4 and P4.5 as inputs (1100 1111)
    P4->REN = EXT_SW; //Enable pullup/pulldown resistor for input pints (0011 0000)
    P4->OUT = EXT_SW; //Resistor is set to pullup for input pins (0011 0000)

    P1->DIR = ~SW; //Set P1.1 and P1.4 as inputs
    P1->REN = SW; //Enable PU/PD
    P1->OUT = SW; //Enable Pullup
}

void output_config(void)
{
    P5->DIR = LEDS; //Set P5.7, P5.6, P5.2, P5.1, P5.0, as outputs
    P2->DIR = 0xF8;
    P2->OUT = 0x00;
    P5->OUT = 0x00;
}

void IRQ_config(void)
{
    //Clear Flag
        P4->IFG &= ~BIT1;
        P1->IFG &= ~BIT1;

    //Interrupt Transition Config
        P4->IES &= BIT1;
        P1->IES &= BIT1;

    //Enable interrupt within peripheral
        P4->IE |= EXT_SW;
        P1->IE |= SW;

    //Enable IRQ
        __NVIC_EnableIRQ(PORT4_IRQn);
        __NVIC_EnableIRQ(PORT1_IRQn);

    //Enable Interrupts
        __enable_irq();
}

void lvl_1(void)
{
    int i = 0;
    sm_delay();
    while(counter == 1)
    {
      if(counter == 1){
      P2->OUT |= BIT3;
      }
          for(i = 150000; i>0; i--);
      if(counter == 1){
      P2->OUT ^= BIT3;
      P5->OUT |= BIT2;
      }
          for(i = 150000; i>0; i--);
      if(counter == 1){
      P5->OUT ^= BIT2;
      }
    }

    P5->OUT |= Rout_mem;
    P2->OUT |= Lout_mem;
}

void lvl_2(void)
{
    int i = 0;
    sm_delay();
    while(counter == 2)
    {
      if(counter == 2){
      P2->OUT |= BIT4;
      }
          for(i = 130000; i>0; i--);
      if(counter == 2){
      P2->OUT ^= BIT4;
      P5->OUT |= BIT1;
      }
          for(i = 130000; i>0; i--);
      if(counter == 2){
      P5->OUT ^= BIT1;
      }
    }
    sm_delay();
    P5->OUT |= Rout_mem;
    P2->OUT |= Lout_mem;
}

void lvl_3(void)
{
    int i = 0;
    sm_delay();
    while(counter == 3)
    {
      if(counter == 3){
      P2->OUT |= BIT5;
      }
          for(i = 100000; i>0; i--);
      if(counter == 3){
      P2->OUT ^= BIT5;
      P5->OUT |= BIT0;
      }
          for(i = 100000; i>0; i--);
      if(counter == 3){
      P5->OUT ^= BIT0;
      }
    }

    P5->OUT |= Rout_mem;
    P2->OUT |= Lout_mem;
}

void lvl_4(void)
{
    int i = 0;
    sm_delay();
    while(counter == 4)
    {
      if(counter == 4){
      P2->OUT |= BIT6;
      }
          for(i = 85000; i>0; i--);
      if(counter == 4){
      P2->OUT ^= BIT6;
      P5->OUT |= BIT6;
      }
          for(i = 85000; i>0; i--);
      if(counter == 4){
      P5->OUT ^= BIT6;
      }
    }

    P5->OUT |= Rout_mem;
    P2->OUT |= Lout_mem;
}


void lvl_5(void)
{
    int i = 0;
    sm_delay();
    while(counter == 5)
    {
      if(counter == 5){
      P2->OUT |= BIT7;
      }
          for(i = 75000; i>0; i--);
      if(counter == 5){
      P2->OUT ^= BIT7;
      P5->OUT |= BIT7;
      }
          for(i = 75000; i>0; i--);
      if(counter == 5){
      P5->OUT ^= BIT7;
      }
    }

    P5->OUT |= Rout_mem;
    P2->OUT |= Lout_mem;
}




void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    __disable_irq();
    __NVIC_DisableIRQ(PORT4_IRQn); //Disable IRQ while configuring
    __NVIC_DisableIRQ(PORT1_IRQn); //Disable IRQ while configuring


    input_config();
    output_config();
    IRQ_config();


    while(1){

           lvl_1();
           sm_delay();
           lvl_2();
           sm_delay();
           lvl_3();
           sm_delay();
           lvl_4();
           sm_delay();
           lvl_5();
    }
}




void PORT4_IRQHandler(void)
{
    sm_delay();

    if((SELECT == sel_TRUE) && (counter > 0)) //Select interrupt
    {
        counter += 1;
        Lout_mem = P2->OUT;
        Rout_mem = P5->OUT;
        P4->IFG = LOW;
    }

    if(RESET == r_TRUE) //reset interrupt
    {
        counter = 0;

        Lout_mem = LOW;
        Rout_mem = LOW;
        P2->OUT = LOW;
        P5->OUT = LOW;

        P4->IFG = LOW;
        P1->IFG = LOW;
    }

   __NVIC_EnableIRQ(PORT4_IRQn);
}

void PORT1_IRQHandler(void)
{
    sm_delay();

    if(START == sta_TRUE) //start interrupt
    {
        if(counter == 0)
        {
            counter+=1;
            P1->IFG = LOW;
            P4->IFG = LOW;
        }
    }

    if(STOP == sto_TRUE) //stop interrupt
    {

    }

   __NVIC_EnableIRQ(PORT1_IRQn);
}

/*
 * gpio.c
 *
 *  Created on: 9/13/2022
 *  Author: Shane McCammon, Frank M, Johnathan T, Ali K
 *  Class: ECEN 2440
 *  Version 3.0
 */

//******************************************
// Include files
//******************************************
#include "gpio.h"

//******************************************
// defined macros
//******************************************

// GPIO: ONBOARD
//-----------------------------------------------------------------------------------------
//      NAME             HEX VALUE  // PHYSICAL         BIN (PINS)      PORT.PIN
//-----------------------------------------------------------------------------------------
#define S1                  0X02    // Onboard SW1      0000 0010       P1.1
#define S2                  0X10    // Onboard SW2      0001 0000       P1.4
#define RED                 BIT0    // Onboard LED      0000 0001       P2.0
#define GREEN               BIT1    // Onboard LED      0000 0010       P2.1
#define BLUE                BIT2    // Onboard LED      0000 0100       P2.2
#define PAUSE               ((P1->IN & S1) == LOW)      // Pause switch: Active LOW


// GPIO: EXTERNAL
//-------------------------------------------------------------------------------------------------------------------
//      NAME                HEX VALUE                   // PHYSICAL         BIN (PINS)       PORTS
//-------------------------------------------------------------------------------------------------------------------
#define RLEDS               (uint8_t)(0xC7)             // Right LEDs:      1100 0111    P5.7, P5.6, P5.2 - P5.0
#define LLEDS               (uint8_t)(0xF8)             // Left LEDs:       1111 1000    P2.7 - P2.3
#define SW_MASK             (uint8_t)(0x02)             // Switches:        0000 0010
#define EXT_SW              (uint8_t)(0x30)             // (0011 0000)
#define OB_SW               (uint8_t)(0x12)             // (0001 0010)
#define START               ((P1->IFG & BIT1) == BIT1)  // Start button condition: Active Low
#define SELECT              ((P4->IFG & BIT5) == BIT5)  // Select button condition: Active Low
#define STOP                ((P1->IFG & BIT4) == BIT4)  // Stop button pressed (IRQ)
#define RESET               ((P4->IFG & BIT4) == BIT4)  // Reset button pressed (IRQ)

// STATES
//-----------------------------------------------------------------------------------------
//      STATE            HEX VALUE              // DESCRIPTION      BIN (PINS)      PORTS
//-----------------------------------------------------------------------------------------
#define LOW                 (uint8_t)(0x00)     // Turn off bit0:   0000 0000       Any
#define HIGH                (uint8_t)(0X01)     // Turn on bit1:    0000 0001       Any
#define LED_OFF             (uint8_t)(0x00)     // Turn OFF LEDs:   0000 0000       P5 & P2
#define LEVEL               FSM->CURR_STATE     // Current state    N/A             N/A
#define SAVE_LEFT           FSM->MEM_LEFT       //
#define SAVE_RIGHT          FSM->MEM_RIGHT      //


// DELAYS
//---------------------- -------------------------------------------------------------------
//      NAME               HEX VALUE // DESCRIPTION                     DECIMAL
//-----------------------------------------------------------------------------------------
#define POLLING_DELAY       0x249F0  // Delay in polling for an input   150,000
#define LVL1_DELAY          0x249F0  // Level 1 delay timer             150,000
#define LVL2_DELAY          0x222E0  // Level 2 delay timer             140,000
#define LVL3_DELAY          0X1D4C0  // Level 3 delay timer             120,000
#define LVL4_DELAY          0X1ADB0  // Level 4 delay timer             110,000
#define LVL5_DELAY          0X15F90  // Level 5 delay timer              90,000
#define SM_DELAY            0X02710  // SM delay timer                   10,000


//******************************************
// function definitions
//******************************************

void gpio_toggle_red_led(void)
{
    P1->DIR |= BIT0;        // P1.0 set as output
    P1->OUT ^= BIT0;        // Blink the P1.0 LED
}


//----------------------------------------------------------------------------------
// STACKER GAME INIT FUNCTIONS
//-----------------------------------------------------------------------------------

void init_game(PORT_E ex_switch,PORT_O ob_switch,
               PORT_E left_leds, PORT_O right_leds,
               uint8_t ex_mask, uint8_t ob_mask,
               uint8_t left_mask, uint8_t right_mask,
               uint8_t flg_bit, IRQn_Type IRQ_ex, IRQn_Type IRQ_ob)
{
    switch_config(ex_switch, ob_switch, ex_mask, ob_mask);
    led_config(left_leds, left_mask, right_leds, right_mask);
    IRQ_config(ex_switch, ob_switch, flg_bit, IRQ_ex, IRQ_ob);
}

void switch_config(PORT_E ex_sw, PORT_O ob_sw, uint8_t ex_mask, uint8_t ob_mask)
{
    ex_sw->DIR = ~ex_mask;      //Set P4.4 and P4.5 as inputs (1100 1111)
    ex_sw->REN = ex_mask;       //Enable pullup/pulldown resistor for input pints (0011 0000)
    ex_sw->OUT = ex_mask;       //Resistor is set to pullup for input pins (0011 0000)

    ob_sw->DIR = ~ob_mask;      //Set P1.1 and P1.4 as inputs
    ob_sw->REN = ob_mask;       //Enable PU/PD
    ob_sw->OUT = ob_mask;       //Enable Pullup
}

void led_config(PORT_E left_leds, uint8_t left_mask,
                PORT_O right_leds, uint8_t right_mask)
{

    left_leds->DIR = left_mask;     // Set P2.7 - P2.3 as output
    left_leds->OUT = left_mask;     // Turn off LEDs
    right_leds->DIR = right_mask;   // Set P5.7, P5.6, P5.2, P5.1, & P5.0 as outputs
    right_leds->OUT = right_mask;   // Turn off LEDs
}


void IRQ_config(PORT_E ex_sw, PORT_O ob_sw,
                uint8_t flg_bit, IRQn_Type IRQ_ex, IRQn_Type IRQ_ob)
{
    //Clear Flag
    ex_sw->IFG &= ~flg_bit;    // BIT1
    ob_sw->IFG &= ~flg_bit;    // BIT1

    //Interrupt edge selector set for positive edge
    ex_sw->IES &= flg_bit;     // BIT1
    ob_sw->IES &= flg_bit;     // BIT1

    //Enable interrupt within peripheral
    ex_sw->IE |= ex_sw;        // EX_SW
    ob_sw->IE |= ob_sw;        // SW

    //Enable IRQ
    __NVIC_EnableIRQ(IRQ_ex);   // PORT4_IRQn
    __NVIC_EnableIRQ(IRQ_ob);   // PORT1_IRQn

    //Enable Interrupts
    __enable_irq();
}

//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------


/*
void stacker_game(void)
{
    // memory variables
    int mem_a, mem_b;

    // Enable input with PU/PD resistor
    enable_pu_pd(P4, SW_MASK);

    // Configure LEDs
    config_leds(P2, LLEDS, P5, RLEDS, LED_OFF);

    // infinite game loop
    while(1)
    {
        // if stop button is pressed, exit while loop
        if(STOP)
        {
            break;
        }

        // if start button is press, begin finite state machine
        if(START)
        {
            // LEVEL 1
            while(SELECT)
            {
                if(set_level(P4, BIT5, P2, BIT3, P5, BIT2, &mem_a, &mem_b, LVL1_DELAY))
                {
                    break;
                }
            }

            change_level(P2, BIT3, P5, BIT2, &mem_a, &mem_b, POLLING_DELAY);

            // LEVEL 2
            while(SELECT)
            {
                if(set_level(P4, BIT5, P2, BIT4, P5, BIT1, &mem_a, &mem_b, LVL2_DELAY))
                {
                    break;
                }
            }

            change_level(P2, BIT4, P5, BIT1, &mem_a, &mem_b, POLLING_DELAY);

            // LEVEL 3
            while(SELECT)
            {
                if(set_level(P4, BIT5, P2, BIT5, P5, BIT0, &mem_a, &mem_b, LVL3_DELAY))
                {
                    break;
                }
            }

            change_level(P2, BIT5, P5, BIT0, &mem_a, &mem_b, POLLING_DELAY);

            // LEVEL 4
            while(SELECT)
            {
                if(set_level(P4, BIT5, P2, BIT6, P5, BIT6, &mem_a, &mem_b, LVL4_DELAY))
                {
                    break;
                }
            }

            change_level(P2, BIT6, P5, BIT6, &mem_a, &mem_b, POLLING_DELAY);

            // LEVEL 5
            while(SELECT)
            {
                if(set_level(P4, BIT5, P2, BIT7, P5, BIT7, &mem_a, &mem_b, LVL5_DELAY))
                {
                    break;
                }
            }

            change_level(P2, BIT7, P5, BIT7, &mem_a, &mem_b, POLLING_DELAY);

            // WINNER! Wait for reset!
            reset(P4, BIT4, P2, P5, LED_OFF);
        }
    }
}
*/



void input_config(PORT_E ex_sw, PORT_O ob_sw, uint8_t ex_mask, uint8_t ob_mask)
{
    ex_sw->DIR = ~ex_mask;
    ex_sw->REN = ex_mask;
    ex_sw->OUT = ex_mask;

    ob_sw->DIR = ~ob_mask;       //Set P1.1 and P1.4 as inputs
    ob_sw->REN = ob_mask;        //Enable port resistor
    ob_sw->OUT = ob_mask;        //Enable pullup
}



void config_leds(PORT_E left_leds_port, uint16_t left_leds_mask,
                 PORT_O right_leds_port, uint16_t right_leds_mask,
                 uint16_t turn_off)
{
    left_leds_port->DIR = left_leds_mask;       // set left LED pins as output
    right_leds_port->DIR = right_leds_mask;     // set right LED pins as output
    left_leds_port->OUT = turn_off;             // turn off left LEDs
    right_leds_port->OUT = turn_off;            // turn off right LEDs
}



void alternate_LEDs(PORT_E led_port, uint16_t led_pin,
                    FSM state_machine, uint32_t delay_time)
{
    delay(delay_time);                               // delay LED signal

    led_port->OUT |= led_pin;                       // Turn on LED
    *state_machine->MEM_LEFT = led_port->OUT;       // save LED state

    delay(delay_time);             // Keep LED on for set time

    led_port->OUT ^= led_pin;      // Turn off LED
    *state_machine->MEM_RIGHT = led_port->OUT;      // save LED state
}

void change_level(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b,
                  uint32_t delay_time)
{
    led_selector(left_led, left_pin, right_led, right_pin, memory_a, memory_b);
    delay(delay_time);
}

/*
void led_selector(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b)
{
    // turn on left LED
    if(memory_b > 0)
    {
        left_led->OUT |= left_pin;
    }

    // turn on right LED
    if(memory_a > 0)
    {
        right_led->OUT |= right_pin;
    }
}
*/



void delay(uint32_t delay_time)
{
    volatile uint32_t i;

    // crude timer
    for(i = delay_time; i > 0; i--);
}


/*
void reset(PORT_E left_leds, PORT_O right_leds, uint16_t reset)
{
    while(1)
    {
        // if reset button pressed, reset game state
        if((reset_btn->IN & btn_pin) != btn_pin)
        {
            left_leds->OUT = reset;     // turn off all left LEDs
            right_leds->OUT = reset;    // turn off all right LEDs
            break;
        }
    }
}
*/

//----------------------------------------------------------------------------------
// IRQ Handlers
//-----------------------------------------------------------------------------------


void PORT1_IRQHandler(void)
{
    if(START) //start interrupt
    {
        if(counter == 0)
        {
            counter+=1;
            P1->IFG &= ~BIT1;
            P4->IFG = LOW;
        }
    }

    if(STOP == sto_TRUE) //stop interrupt
    {

    }

   __NVIC_EnableIRQ(PORT1_IRQn);
}

void PORT4_IRQHandler(void)
{
    if(SELECT) //Select interrupt
    {
        counter += 1;
        Lout_mem = P2->OUT;
        Rout_mem = P5->OUT;
        P4->IFG = LOW;
    }

    if(RESET) //reset interrupt
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

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
#define START               ((P1->IFG & BIT1) == BIT1)  // Start button pressed (IRQ)
#define STOP                ((P1->IFG & BIT4) == BIT4)  // Stop button pressed (IRQ)


// GPIO: EXTERNAL
//-------------------------------------------------------------------------------------------------------------------
//      NAME                HEX VALUE                   // PHYSICAL         BIN (PINS)       PORTS
//-------------------------------------------------------------------------------------------------------------------
#define RLEDS               (uint8_t)(0xC7)             // Right LEDs:      1100 0111    P5.7, P5.6, P5.2 - P5.0
#define LLEDS               (uint8_t)(0xF8)             // Left LEDs:       1111 1000    P2.7 - P2.3
#define EXT_SW_MASK         (uint8_t)(0x30)             // (0011 0000)
#define OB_SW_MASK          (uint8_t)(0x12)             // (0001 0010)
#define SELECT              ((P4->IFG & BIT5) == BIT5)  // Select button pressed (IRQ)
#define RESET               ((P4->IFG & BIT4) == BIT4)  // Reset button pressed (IRQ)

// STATES
//---------------------------------------------------------------------------------------------
//      STATE            HEX VALUE              // DESCRIPTION          BIN (PINS)      PORTS
//---------------------------------------------------------------------------------------------
#define CLEAR               (uint8_t)(0x00)     // Clear memory         0000 0000       N/A
#define LOW                 (uint8_t)(0x00)     // Turn off bit0:       0000 0000       Any
#define HIGH                (uint8_t)(0X01)     // Turn on bit1:        0000 0001       Any
#define LEDS_OFF            (uint8_t)(0x00)     // Turn OFF LEDs:       0000 0000       P5 & P2

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
// static variables
//******************************************
static FSM_Game_Type* GAME;


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

void init_game(void)
{
    game_config();
    switch_config();
    led_config();
    IRQ_config();
}

void game_config(void)
{
    GAME->currentLevel = LEVEL0;
    GAME->previousLevel = CLEAR;
    GAME->leftMemory = CLEAR;
    GAME->rightMemory = CLEAR;
    GAME->leftPin = CLEAR;
    GAME->rightPin = CLEAR;
}

void switch_config(void)
{
    P4->DIR = ~EXT_SW_MASK;      //Set P4.4 and P4.5 as inputs (1100 1111)
    P4->REN = EXT_SW_MASK;       //Enable pullup/pulldown resistor for input pints (0011 0000)
    P4->OUT = EXT_SW_MASK;       //Resistor is set to pullup for input pins (0011 0000)

    P1->DIR = ~OB_SW_MASK;      //Set P1.1 and P1.4 as inputs
    P1->REN = OB_SW_MASK;       //Enable PU/PD
    P1->OUT = OB_SW_MASK;       //Enable Pullup
}

void led_config(void)
{

    P2->DIR = LLEDS;    // Set P2.7 - P2.3 as output
    P2->OUT = LEDS_OFF;  // Turn off LEDs
    P5->DIR = RLEDS;    // Set P5.7, P5.6, P5.2, P5.1, & P5.0 as outputs
    P5->OUT = LEDS_OFF;  // Turn off LEDs
}


void IRQ_config(void)
{
    //Clear Flag
    P4->IFG &= ~BIT1;
    P1->IFG &= ~BIT1;

    //Interrupt edge selector set for positive edge
    P4->IES &= BIT1;
    P1->IES &= BIT1;

    //Enable interrupt within peripheral
    P4->IE |= EXT_SW_MASK;       // external switches
    P1->IE |= OB_SW_MASK;        // onboard switches

    //Enable IRQ
    __NVIC_EnableIRQ(PORT4_IRQn);
    __NVIC_EnableIRQ(PORT1_IRQn);

    //Enable Interrupts
    __enable_irq();
}

//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------


void stacker_game(void)
{

}


void alternate_LEDs(uint8_t left_led_pin, uint8_t right_led_pin, uint32_t delay_time)
{
    delay(delay_time);              // delay LED signal

    P2->OUT |= left_led_pin;        // turn on left LED
    GAME->leftMemory = P2->OUT;     // dave left LED state

    delay(delay_time);              // keep left LED on for set time

    P2->OUT ^= left_led_pin;        // turn off left LED
    GAME->leftMemory = P2->OUT;     // save left LED state

    delay(delay_time);              // delay LED signal

    P5->OUT |= right_led_pin;       // turn on right LED
    GAME->rightMemory = P5->OUT;    // save LED state

    delay(delay_time);              // keep right LED on for set time

    P5->OUT ^= right_led_pin;       // turn off right LED
    GAME->rightMemory = P5->OUT;    // save right LED state
}

void change_level(LEVELn_Type next_level, uint8_t left_pin, uint8_t right_pin)
{
    GAME->currentLevel = next_level;
    GAME->leftPin =  left_pin;
    GAME->rightPin = right_pin;
}


void delay(uint32_t delay_time)
{
    volatile uint32_t i;

    // crude timer
    for(i = delay_time; i > 0; i--);
}


void start(void)
{
    // start functionality
}


void reset(void)
{
    // clear Game memory registers
    GAME->previousLevel = CLEAR;
    GAME->leftMemory = CLEAR;
    GAME->rightMemory = CLEAR;
    GAME->leftPin = CLEAR;
    GAME->rightPin = CLEAR;
}


void game_over(void)
{

}

//----------------------------------------------------------------------------------
// IRQ Handlers
//-----------------------------------------------------------------------------------


void PORT1_IRQHandler(void)
{
    // start button interrupt source
    if(START)
    {
        // if game not started
        if(GAME->currentLevel == LEVEL0)
        {
            // GOTO: level 1
            change_level(LEVEL1, BIT3, BIT2);
        }
    }

    // stop button interrupt source
    if(STOP)
    {
        // GOTO: level 0 (off)
        change_level(LEVEL0, LEDS_OFF, LEDS_OFF);
    }

    // lower flag
    P1->IFG &= ~BIT1;

    // re-enable NVIC interrupt
   __NVIC_EnableIRQ(PORT1_IRQn);
}

void PORT4_IRQHandler(void)
{
    // select interrupt source
    if(SELECT)
    {
        switch(GAME->currentLevel)
        {
            case LEVEL1:
                // next state
                break;

            case LEVEL2:
                // next state
                break;

            case LEVEL3:
                // next state
                break;

            case LEVEL4:
                // next state
                break;

            case LEVEL5:
                // next state
                break;
        }
    }

    // reset interrupt source
    if(RESET)
    {
        if(GAME->currentLevel > LEVEL0)
        {
            // reset game memory registers
            reset();

            // GOTO: level 1
            change_level(LEVEL1, BIT3, BIT2);
        }
    }

    // lower flag
    P4->OUT &= ~BIT1;

    // re-enable NVIC Interrupt
   __NVIC_EnableIRQ(PORT4_IRQn);
}

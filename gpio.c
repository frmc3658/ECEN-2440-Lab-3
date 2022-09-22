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
#define LVL1_DELAY          0x222E0  // Level 1 delay timer             140,000
#define LVL2_DELAY          0x1FBD0  // Level 2 delay timer             130,000
#define LVL3_DELAY          0x1C138  // Level 3 delay timer             115,000
#define LVL4_DELAY          0x186A0  // Level 4 delay timer             100,000
#define LVL5_DELAY          0x15F90  // Level 5 delay timer              90,000
#define SM_DELAY            0x03A98  // SM delay timer                   15,000


//******************************************
// static variables
//******************************************
static FSM_Game_Type GAME;
static uint8_t lvl_counter = 2;

//******************************************
// function definitions
//******************************************


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
    GAME.currentLevel = LEVEL0;
    GAME.previousLevel = CLEAR;
    GAME.leftMemory = CLEAR;
    GAME.rightMemory = CLEAR;
    GAME.leftPin = CLEAR;
    GAME.rightPin = CLEAR;
}

void switch_config(void)
{
    P4->DIR = ~EXT_SW_MASK;     //Set P4.4 and P4.5 as inputs (1100 1111)
    P4->REN = EXT_SW_MASK;      //Enable pullup/pulldown resistor for input pints (0011 0000)
    P4->OUT = EXT_SW_MASK;      //Resistor is set to pullup for input pins (0011 0000)

    P1->DIR = ~OB_SW_MASK;      //Set P1.1 and P1.4 as inputs
    P1->REN = OB_SW_MASK;       //Enable PU/PD
    P1->OUT = OB_SW_MASK;       //Enable Pullup
}

void led_config(void)
{

    P2->DIR = LLEDS;    // Set P2.7 - P2.3 as output
    P2->OUT = LEDS_OFF; // Turn off LEDs
    P5->DIR = RLEDS;    // Set P5.7, P5.6, P5.2, P5.1, & P5.0 as outputs
    P5->OUT = LEDS_OFF; // Turn off LEDs
}


void IRQ_config(void)
{
    //Clear Flag
    P4->IFG = CLEAR;
    P1->IFG = CLEAR;

    //Interrupt edge selector set for positive edge
    P4->IES = 0b00110000;
    P1->IES = 0b00010010;

    //Enable interrupt within peripheral
    P4->IE = EXT_SW_MASK;       // external switches
    P1->IE = OB_SW_MASK;        // onboard switches

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
    if(GAME.currentLevel > LEVEL0)
    {
        uint8_t leftLED = GAME.leftPin;
        uint8_t rightLED = GAME.rightPin;
        uint32_t delay = GAME.delay;

        alternate_LEDs(leftLED, rightLED, delay);
    }
}


void alternate_LEDs(uint8_t left_led_pin, uint8_t right_led_pin, uint32_t delay_time)
{
    GAME.ledCounter = 0;
    uint8_t lvl_check = GAME.currentLevel;
    delay(SM_DELAY);

    if(GAME.currentLevel == lvl_check){
    P2->OUT |= left_led_pin;        // turn on left LED
    GAME.leftMemory = P2->OUT;     // save left LED state
    GAME.ledCounter++;
    update_OUT();
    delay(delay_time);              // delay LED signal
    }


    if(GAME.currentLevel == lvl_check){
    P2->OUT ^= left_led_pin;        // turn off left LED
    GAME.leftMemory = P2->OUT;     // save left LED state
    GAME.ledCounter++;
    update_OUT();
    delay(delay_time);              // delay LED signal
    }

    if(GAME.currentLevel == lvl_check){
    P5->OUT |= right_led_pin;       // turn on right LED
    GAME.rightMemory = P5->OUT;    // save LED state
    GAME.ledCounter++;
    update_OUT();
    delay(delay_time);              // delay LED signal
    }

    if(GAME.currentLevel == lvl_check){
    P5->OUT ^= right_led_pin;       // turn off right LED
    GAME.rightMemory = P5->OUT;    // save right LED state
    GAME.ledCounter++;
    update_OUT();
    delay(delay_time);              // delay LED signal
    }

}

void change_level(LEVELn_Type next_level, LEVELn_Type previous_level,
                  uint8_t left_pin, uint8_t right_pin, uint32_t delay)
{
    GAME.currentLevel = next_level;
    GAME.previousLevel = previous_level;
    GAME.leftPin =  left_pin;
    GAME.rightPin = right_pin;
    GAME.delay = delay;
}


void delay(uint32_t delay_time)
{
    volatile uint32_t i;

    // crude timer
    for(i = delay_time; i > 0; i--);
}

void update_OUT(void)
{
    P5->OUT |= GAME.rightProgress;
    P2->OUT |= GAME.leftProgress;
}


void reset(void)
{
    // clear Game memory registers
    GAME.currentLevel = CLEAR;
    GAME.previousLevel = CLEAR;
    GAME.leftMemory = CLEAR;
    GAME.rightMemory = CLEAR;
    GAME.leftPin = CLEAR;
    GAME.rightPin = CLEAR;
    GAME.delay = LVL1_DELAY;
    GAME.rightProgress = CLEAR;
    GAME.leftProgress = CLEAR;
    P5->OUT = CLEAR;
    P2->OUT = CLEAR;
}


void game_over(void)
{
    // will program later
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
        if(GAME.currentLevel == LEVEL0)
        {
            // GOTO: level 1
            change_level(LEVEL1, LEVEL0, BIT3, BIT2, LVL1_DELAY);
        }

        // lower flag
        P1->IFG &= ~BIT1;
    }

    // stop button interrupt source
    if(STOP)
    {
        // GOTO: level 0 (off)
        change_level(LEVEL0, LEVEL0, LEDS_OFF, LEDS_OFF, LVL1_DELAY);

        // lower flag
        P1->IFG &= ~BIT4;
    }

    // re-enable NVIC interrupt
   __NVIC_EnableIRQ(PORT1_IRQn);
}

void PORT4_IRQHandler(void)
{
    // select interrupt source
    delay(SM_DELAY);
    if(SELECT)
    {
        delay(SM_DELAY);
        if(GAME.currentLevel > LEVEL0)
        {
            switch(GAME.currentLevel)
            {
                case LEVEL1:
                    change_level(LEVEL2, LEVEL1, BIT4, BIT1, LVL2_DELAY);
                    break;

                case LEVEL2:
                    change_level(LEVEL3, LEVEL2, BIT5, BIT0, LVL3_DELAY);
                    break;

                case LEVEL3:
                    change_level(LEVEL4, LEVEL3, BIT6, BIT6, LVL4_DELAY);
                    break;

                case LEVEL4:
                    change_level(LEVEL5, LEVEL4, BIT7, BIT7, LVL5_DELAY);
                    break;

                case LEVEL5:
                    change_level(FINISH, LEVEL5, LLEDS, RLEDS, SM_DELAY);
                    break;
            }

            // if counter is even, save right LED ...
            if(GAME.ledCounter > 2)
            {
                GAME.rightProgress = GAME.rightMemory;
            }
            // ... else save left LED...
            else
            {
                GAME.leftProgress = GAME.leftMemory;
            }
            // ... to track progress
            delay(SM_DELAY);
            // lower flag
            P4->IFG &= ~BIT5;
        }

    }

    // reset interrupt source
    if(RESET)
    {
        if(GAME.currentLevel > LEVEL0)
        {
            // reset game memory registers
            reset();

            // GOTO: level 1
            change_level(LEVEL1, LEVEL0, BIT3, BIT2, LVL1_DELAY);
        }

        // lower flag
        P4->IFG = CLEAR;
    }

    // re-enable NVIC Interrupt
   __NVIC_EnableIRQ(PORT4_IRQn);
}

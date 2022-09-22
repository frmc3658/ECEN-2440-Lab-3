/*
 * gpio.h
 *
 *  Created on: 9/13/2022
 *  Author: Shane Mcammon, Frank M, Johnathan T, Ali K
 *  Class: ECEN 2440
 *  Version 3.0
 */

#ifndef GPIO_H_
#define GPIO_H_

//***********************************************************************************
// include files
//***********************************************************************************
#include "msp.h"


//***********************************************************************************
// data structures
//***********************************************************************************


typedef struct
{
    uint8_t ledCounter;
    uint8_t currentLevel;       // memory register to store the current level
    uint8_t previousLevel;      // memory register to store the previous level
    uint8_t leftMemory;         // memory register to store the state of the left LED
    uint8_t rightMemory;        // memory register to store the state of the right LED
    uint8_t leftPin;            // memory register to store current left LED output pin
    uint8_t rightPin;           // memory register to store current right LED output pin
    uint32_t delay;             // memory register to store current delay value
    uint8_t leftProgress;       // memory register to track all of the selected left LEDs
    uint8_t rightProgress;      // memory register to track all of the selected right LEDs
}FSM_Game_Type;


typedef enum LEVELn
{
    LEVEL0           = (uint8_t)(0x01),         /*Bin: 0000 0001*/
    LEVEL1           = (uint8_t)(0x02),         /*Bin: 0000 0010*/
    LEVEL2           = (uint8_t)(0x04),         /*Bin: 0000 0100*/
    LEVEL3           = (uint8_t)(0x08),         /*Bin: 0000 1000*/
    LEVEL4           = (uint8_t)(0x10),         /*Bin: 0001 0000*/
    LEVEL5           = (uint8_t)(0x20),         /*Bin: 0010 0000*/
    FINISH           = (uint8_t)(0x40),         /*Bin: 0100 0000*/
    GAME_OVER        = (uint8_t)(0x80)          /*Bin: 1000 0000*/
}LEVELn_Type;


//***********************************************************************************
// function prototypes
//***********************************************************************************

//----------------------------------------------------------------------------------
// STACKER GAME INIT FUNCTIONS
//-----------------------------------------------------------------------------------

void init_game(void);

void game_config(void);

void switch_config(void);

void led_config(void);

void IRQ_config(void);


//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------

void stacker_game(void);

void alternate_LEDs(uint8_t left_led_pin, uint8_t right_led_pin, uint32_t delay_time);

void change_level(LEVELn_Type next_level, LEVELn_Type previous_level,
                  uint8_t left_pin, uint8_t right_pin, uint32_t delay);


void delay(uint32_t delay_time);

void reset(void);

void game_over(void);

#endif /* GPIO_H_ */

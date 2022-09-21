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
// Include files
//***********************************************************************************x
#include "msp.h"


//**********************************************************************************************************
// defined files
//**********************************************************************************************************
#define PORT_E      DIO_PORT_Even_Interruptable_Type*     // Provides access to Ports 4 and 2
#define PORT_O      DIO_PORT_Odd_Interruptable_Type*      // Provides access to Ports 5 and 1
#define FSM         FSM_Game_Type*                        // Provides access to Finite State Machine Struct

typedef struct
{
    uint8_t CURR_STATE;
    uint8_t MEM_LEFT;
    uint8_t MEM_RIGHT;
}FSM_Game_Type;


typedef enum
{
    GAME_OVER        = -1,
    LEVEL0           = 0,
    LEVEL1           = 1,
    LEVEL2           = 2,
    LEVEL3           = 3,
    LEVEL4           = 4,
    LEVEL5           = 5,
    FINISH           = 6
}LEVELn_Type;

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

void gpio_toggle_red_led(void);

//----------------------------------------------------------------------------------
// STACKER GAME INIT FUNCTIONS
//-----------------------------------------------------------------------------------

void init_game(PORT_E ex_switch,PORT_O ob_switch,
               PORT_E left_leds, PORT_O right_leds,
               uint8_t ex_mask, uint8_t ob_mask,
               uint8_t left_mask, uint8_t right_mask,
               uint8_t flg_bit, IRQn_Type IRQ_ex, IRQn_Type IRQ_ob);

void switch_config(PORT_E ex_sw, PORT_O ob_sw, uint8_t ex_mask, uint8_t ob_mask);

void led_config(PORT_E left_leds, uint8_t left_mask,
                PORT_O right_leds, uint8_t right_mask);

void IRQ_config(PORT_E ex_sw, PORT_O ob_sw,
                uint8_t flg_bit, IRQn_Type IRQ_ex, IRQn_Type IRQ_ob);


//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------

void stacker_game(void);


void config_leds(PORT_E left_leds_port, uint16_t left_leds_mask,
                 PORT_O right_leds_port, uint16_t right_leds_mask,
                 uint16_t turn_off);



void alternate_LEDs(PORT_E led_port, uint16_t led_pin,
                    FSM state_machine, uint32_t delay_time);

void change_level(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b,
                  uint32_t delay_time);


void led_selector(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b);



void delay(uint32_t delay_time);


void reset(PORT_E left_leds, PORT_O right_leds, uint8_t reset);


//----------------------------------------------------------------------------------
// IRQ Handlers
//-----------------------------------------------------------------------------------

void PORT1_IRQHandler(void);

void PORT4_IRQHandler(void);

#endif /* GPIO_H_ */

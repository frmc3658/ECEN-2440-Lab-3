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


//***********************************************************************************
// defined files
//***********************************************************************************
#define PORT_E        DIO_PORT_Even_Interruptable_Type*     // Provides access to Ports 4 and 2 (which are even)
#define PORT_O        DIO_PORT_Odd_Interruptable_Type*      // Provides access to Port 5 (Which is odd)

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

void gpio_toggle_red_led(void);


//----------------------------------------------------------------------------------
// RGB DANCE PARTY GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------

void gpio_rgb_dance_party(void);


void dance_party_delay(uint32_t dance_party_delay);


//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------


void stacker_game(void);



void enable_pu_pd(PORT_E switch_port, uint16_t sw_mask);



void config_leds(PORT_E left_leds_port, uint16_t left_leds_mask,
                 PORT_O right_leds_port, uint16_t right_leds_mask,
                 uint16_t turn_off);



int set_level(PORT_E select_btn, uint16_t btn_pin,
              PORT_E left_led, uint16_t left_pin,
              PORT_O right_led, uint16_t right_pin,
              int* memory_a, int* memory_b,
              uint32_t delay_time);

void change_level(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b,
                  uint32_t delay_time);


void led_selector(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b);



void delay(uint32_t delay_time);



void reset(PORT_E reset_btn, uint16_t btn_pin,
           PORT_E left_leds, PORT_O right_leds,
           uint16_t reset);



#endif /* GPIO_H_ */

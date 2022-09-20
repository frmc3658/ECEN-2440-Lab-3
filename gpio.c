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
//      NAME             HEX VALUE  // PHYSICAL         BIN (PINS)      PORTS
//-----------------------------------------------------------------------------------------
#define S1                  0X02    // Onboard SW1      0000 0010       P1.1
#define S2                  0X10    // Onboard SW2      0001 0000       P1.4
#define RED                 BIT0    // Onboard LED      0000 0001       P2.0
#define GREEN               BIT1    // Onboard LED      0000 0010       P2.1
#define BLUE                BIT2    // Onboard LED      0000 0100       P2.2
#define RB                  0x05    // Onboard LED      0000 0101       P2.0 & P2.2
#define GB                  0x06    // Onboard LED      0000 0110       P2.1 - P2.2
#define RGB                 0x07    // Onboard LED      0000 0111       P2.0 - P2.3
#define RG                  0x03    // Onboard LED      0000 0011       P2.0 - P2.1
#define DANCE_OFF           0x00    // Turn off         0000 0000       P2.0 - 2.7
#define PAUSE               ((P1->IN & S1) == LOW)      // Pause switch: Active LOW


// GPIO: EXTERNAL
//-----------------------------------------------------------------------------------------
//      NAME             HEX VALUE  // PHYSICAL         BIN (PINS)       PORTS
//-----------------------------------------------------------------------------------------
#define RLEDS               0xC7    // Right LEDs:      1100 0111    P5.7, P5.6, P5.2 - P5.0
#define LLEDS               0xF8    // Left LEDs:       1111 1000    P2.7 - P2.3
#define SW_MASK             0x02    // Switches:        0000 0010
#define START               ((P4->IN & BIT0) != BIT0)   // Start button condition: Active Low
#define SELECT              ((P4->IN & BIT5) == BIT5)   // Select button condition: Active Low
#define STOP                ((P4->IN & BIT2) != BIT2)   // Stop button pressed

// STATES
//-----------------------------------------------------------------------------------------
//      STATE            HEX VALUE  // DESCRIPTION      BIN (PINS)      PORTS
//-----------------------------------------------------------------------------------------
#define LOW                 0x00    // Turn off bit0:   0000 0000
#define HIGH                0X01    // Turn on bit1:    0000 0001
#define OFF                 0x00    // Turn OFF LEDs:   0000 0000       P5 & P2
#define POLL_SELECT         ((P4->IN & BIT5) != BIT5)   // Poll select button


// DELAYS
//-----------------------------------------------------------------------------------------
//      NAME               HEX VALUE // DESCRIPTION                     DECIMAL
//-----------------------------------------------------------------------------------------
#define DANCE_DELAY_TIMER   0X186A0  // Delay for RGB Dance Party       100,000
#define POLLING_DELAY       0x249F0  // Delay in polling for an input   150,000
#define LVL1_DELAY          0x249F0  // Level 1 delay timer             150,000
#define LVL2_DELAY          0x222E0  // Level 2 delay timer             140,000
#define LVL3_DELAY          0X1D4C0  // Level 3 delay timer             120,000
#define LVL4_DELAY          0X1ADB0  // Level 4 delay timer             110,000
#define LVL5_DELAY          0X15F90  // Level 5 delay timer              90,000


//******************************************
// function definitions
//******************************************

void gpio_toggle_red_led(void)
{
    P1->DIR |= BIT0;        // P1.0 set as output
    P1->OUT ^= BIT0;        // Blink the P1.0 LED
}

//----------------------------------------------------------------------------------
// RGB DANCE PARTY FUNCTION DEFINITIONS
//-----------------------------------------------------------------------------------

void gpio_rgb_dance_party(void)
{
    P2->DIR |= RED;         // Set P2.0 (RED LED) as output
    P2->DIR |= GREEN;       // Set P2.1 (GREEN LED) as output
    P2->DIR |= BLUE;        // Set P2.2 (BLUE LED) as ouput

    P1->REN |= S1;          // Enable Resistor; P1.1 (S1)
    P1->REN |= S2;          // Enable Resistor; P1.4 (S2)
    P1->DIR |= S1;          // Set P1.1 (S1) as output
    P1->DIR |= S2;          // Set P1.4 (S2) as output

    // infinite loop
    while(1)
    {
        //
        if((P1->IN & S2) == LOW)
        {
            while(1)
            {
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = BLUE;
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = RB;
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = GB;
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = RGB;
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = RG;
                dance_party_delay(DANCE_DELAY_TIMER);
                P2->OUT = RED;
            }
        }
        else
        {
            P2->OUT = DANCE_OFF;
        }
    }
}


void dance_party_delay(uint32_t dance_party_delay)
{
    volatile uint32_t i;

    // DELAY STATE
    for(i = dance_party_delay; i > 0; i--)
    {
        // if S1 is pressed, enter PAUSE state
        if(PAUSE)
        {
            // When S1 is released, RESUME LED cycle
            while(PAUSE)
            {
                /* if the intended functionality is to turn OFF the LED
                /  when paused, uncomment the line below. With the line commented out
                /  while S1 is held down it will pause, leaving the previous light
                /  illuminated.*/
                //P2->OUT = DANCE_PARTY_LED_OFF;
            }
        }
    }
}



//----------------------------------------------------------------------------------
// STACKER GAME FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------

void stacker_game(void)
{
    // memory variables
    int mem_a, mem_b;

    // Enable input with PU/PD resistor
    enable_pu_pd(P4, SW_MASK);

    // Configure LEDs
    config_leds(P2, LLEDS, P5, RLEDS, OFF);

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
            reset(P4, BIT4, P2, P5, OFF);
        }
    }
}



void enable_pu_pd(PORT_E switch_port, uint16_t sw_mask)
{
    switch_port->DIR = sw_mask;
    switch_port->REN = ~sw_mask;
    switch_port->OUT = ~sw_mask;
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



int set_level(PORT_E select_btn, uint16_t btn_pin,
              PORT_E left_led, uint16_t left_pin,
              PORT_O right_led, uint16_t right_pin,
              int* memory_a, int* memory_b,
              uint32_t delay_time)
{
    left_led->OUT |= left_pin;  //Turn on left
    *memory_a = left_led->OUT;   //memory

    // delay
    delay(delay_time);

    // If select button pressed, exit
    if((select_btn->IN & btn_pin) != btn_pin)
    {
        return 1;
    }

    left_led->OUT ^= left_pin;      // Turn off left LED
    *memory_a = left_led->OUT;       // Memory
    right_led->OUT |= right_pin;    // Turn on right LED
    *memory_b = right_led->OUT;      // Memory

    // delay
    delay(delay_time);

    // If select button pressed, exit
    if((select_btn->IN & btn_pin) != btn_pin)
    {
        return 1;
    }

    right_led->OUT ^= right_pin;    // Turn off right LED
    *memory_b = right_led->OUT;      // Memory

    return 0;
}

void change_level(PORT_E left_led, uint16_t left_pin,
                  PORT_O right_led, uint16_t right_pin,
                  int* memory_a, int* memory_b,
                  uint32_t delay_time)
{
    led_selector(left_led, left_pin, right_led, right_pin, memory_a, memory_b);
    delay(delay_time);
}


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



void delay(uint32_t delay_time)
{
    volatile uint32_t i;

    // crude timer
    for(i = delay_time; i > 0; i--);
}



void reset(PORT_E reset_btn, uint16_t btn_pin,
           PORT_E left_leds, PORT_O right_leds,
           uint16_t reset)
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

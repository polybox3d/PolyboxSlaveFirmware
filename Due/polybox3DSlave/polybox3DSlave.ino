/*
    This file is part of Polybox.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

/***********************************************************
 * @file slave.ino
 * @author Florian Boudinet <florian.boudinet@gmail.com>
 *
 * Sketch built for ino tool.
 * This sketch setup an I2C slave board to speak with a master.
 * It's a part of Polybox-Firmware.
 *
 * *********************************************************/
#include <inttypes.h>
#include <Wire.h>

#include "SimpleList.h"

#include "Configuration.h"
#include "HAL.h"
#include "eps.h" // Extension Pin System


/***********************************************************************
 *
 * Variables
 *
 * ********************************************************************/
typedef struct PWM_ {
    uint8_t pin;
}PWM_PIN;

uint8_t need_check = true ;
uint32_t timer_check_pin_slow = 0;
uint32_t timer_check_pin_fast = 0;
uint32_t timer_process_analog = 0;
uint8_t current_analog = 0; // current analog pin
uint8_t current_analog_sum_number = 0; // we sum X time each analog value, to smooth result (average result)
volatile byte timer_eps_update = 0 ;// 10ms*10 = 100ms; use base counter
SimpleList<PWM_PIN> pwm_pins;
uint8_t pwm_clk_count = 0;
int pwm_count_clk_count = 0;

/***********************************************************************
 *
 * Fonctions
 *
 * ********************************************************************/
#define WRITE_VAR(pin, v) do{if(v) {g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;} else {g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin; }}while(0)

void setAllPinInput()
{
    #if OUTPUT_SERIAL
    Serial.print(" > INIT INPUT TYPE < ");
    #endif
    for (uint8_t i = 0; i < PINS_PER_BOARD ; ++i )
    {
            board.pin_values[i]->type = PIN_TYPE_INPUT;
    }
}

#define PWM_TIMER               TC0
#define PWM_TIMER_CHANNEL       1
#define PWM_TIMER_IRQ           ID_TC1
#define PWM_TIMER_VECTOR        TC1_Handler
#define PWM_CLOCK_FREQ          124992// that's 488,25Hz //3906
void setupInterrupts()
{
  uint32_t     tc_count, tc_clock;

  pmc_set_writeprotect(false);
    // Regular interrupts for heater control etc
  pmc_enable_periph_clk(PWM_TIMER_IRQ);
  //NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, NVIC_EncodePriority(4, 6, 0));
  NVIC_SetPriority((IRQn_Type)PWM_TIMER_IRQ, 10);

  TC_FindMckDivisor(PWM_CLOCK_FREQ, F_CPU_TRUE, &tc_count, &tc_clock, F_CPU_TRUE);
  TC_Configure(PWM_TIMER, PWM_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | tc_clock);

  TC_SetRC(PWM_TIMER, PWM_TIMER_CHANNEL, (F_CPU_TRUE / tc_count) / PWM_CLOCK_FREQ);
  TC_Start(PWM_TIMER, PWM_TIMER_CHANNEL);

  PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
  PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ((IRQn_Type)PWM_TIMER_IRQ);
}
/**
 * Setup the Slave. Defined and initialize variables,
 * create Serial Connexion and join the I2C BUS using BOARD_ID identifier.
 * Start Analogique timer.
 * **/
void setup()
{
    #if OUTPUT_SERIAL
        Serial.begin( SERIAL_BAUDRATE );
        Serial.print(" START ID : ");
        Serial.print(BOARD_ID);
    #endif
    // We add pin 44, 45, 46 to the pwm_pin list because the DUE board doesn't handle hardware PWM.
    pwm_pins.push_back({44});
    pwm_pins.push_back({45});
    pwm_pins.push_back({46});

    // Setup all the interrupts we need
    setupInterrupts();

    setup_slave_master( );
    startWatchdog();
   // setup_analog_timer();

    // Read and init ADCC for 1st time (dummy value)
    delay( DELAY_START_UP );
   // setAllPinInput();
}

/**
 * Main Function for the µP, called forever (while true).
 * **/
void loop() {

    /**
     * Manage Board and I2C.
     * **/
   // eps_manage();
    if ( board.check_state == BOARD_OFF )
    {
        #if OUTPUT_SERIAL
            Serial.print(" OFF ");
        #endif
        delay( DELAY_OFF );
    }
    else if ( board.check_state == BOARD_W8_MASTER )
    {
        #if OUTPUT_SERIAL
            Serial.print(" W8 ");
        #endif
        delay( DELAY_INIT );
    }
    else
    {
        /**************************************************************
         *
         *  CHECK PIN STATE & PUSH
         *
         * ************************************************************/
        //Check FAST pin ?
        ++timer_check_pin_fast;
        if ( timer_check_pin_slow >= DELAY_CHECK_PIN_FAST )
        {
        //    board.check_pins_update( PIN_TYPE_FAST_CHECK);
            timer_check_pin_fast = 0;
        }
        //Check SLOW (classic) pin ?
        ++timer_check_pin_slow;
        if ( timer_check_pin_slow >= DELAY_CHECK_PIN_SLOW )
        {
            board.check_pins_update( );
            timer_check_pin_slow = 0;
        }
        // push analog update.
        ++timer_process_analog;
        if ( timer_process_analog >= DELAY_PROCESS_ANALOG )
        {
            board.process_analog();
            timer_process_analog  = 0;
        }
        delay( DELAY_EXEC_LOOP );
        #if OUTPUT_SERIAL
            #if OUTPUT_END_OF_LOOP
                Serial.print(".");
            #endif
        #endif
    }
    /*#if OUTPUT_SERIAL
        Serial.print(" ");
        Serial.print(pwm_count_clk_count);
        Serial.print(" ");
    #endif*/
    pingWatchdog();
}


/**
 * Small test for the Arduino Bug on Due (analogWrite() & pinMode() bugs)
        #define TEST_PIN   7
        char response;                        // response from serial in
         //Serial.println("o=OUTPUT, i=INPUT, h=HIGH, l=LOW");
         if ( Serial.available() )
         {           // wait for input
             response = Serial.read();

             switch(response) {
             case 'o':
               Serial.println("Output");
               pinMode(TEST_PIN,OUTPUT);
               break;

             case 'i':
               Serial.println("Input");
               pinMode(TEST_PIN,INPUT);
               break;

             case 'h':
               Serial.println("High");
               analogWrite(TEST_PIN,255);
               break;
            case 'm':
               Serial.println("Medium");
               digitalWrite(TEST_PIN,0);
               analogWrite(TEST_PIN,125);
               break;
            case 'd':
               Serial.println("Digital");
               digitalWrite(TEST_PIN,0);
               break;

             case 'l':
               Serial.println("Low");
               analogWrite(TEST_PIN,0);
               break;

             default:
               break;
             }
         }
**/





/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs.
*/
void PWM_TIMER_VECTOR ()
{
    // apparently have to read status register
    TC_GetStatus(PWM_TIMER, PWM_TIMER_CHANNEL);

    // Check all pwm pin, and set output regarding the current pos of pwm_clk
    for (SimpleList<PWM_PIN>::iterator itr = pwm_pins.begin(); itr != pwm_pins.end(); ++itr)
    {

        if ( pwm_clk_count !=  255 && pwm_clk_count == (255 - board.read_bpin((*itr).pin)) )
        {
            WRITE_VAR( (*itr).pin, HIGH );
        }
        else if ( pwm_clk_count == 0 )
        {
            WRITE_VAR( (*itr).pin, 0 );
        }
    }
    pwm_clk_count++;
}







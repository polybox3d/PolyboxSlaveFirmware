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

#include "Configuration.h"
#include "HAL.h"
#include "eps.h" // Extension Pin System


/***********************************************************************
 *
 * Variables
 *
 * ********************************************************************/
uint8_t need_check = true ;
uint32_t timer_check_pin_slow = 0;
uint32_t timer_check_pin_fast = 0;
uint32_t timer_process_analog = 0;
uint8_t current_analog = 0; // current analog pin
uint8_t current_analog_sum_number = 0; // we sum X time each analog value, to smooth result (average result)
volatile byte timer_eps_update = 0 ;// 10ms*10 = 100ms; use base counter



/***********************************************************************
 *
 * Fonctions
 *
 * ********************************************************************/

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

    startWatchdog();

    setup_slave_master( );
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
    }
    pingWatchdog();
}


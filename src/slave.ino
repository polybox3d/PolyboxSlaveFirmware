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

#include "Configuration_eps.h"
#include "eps.h" // Extension Pin System

/***********************************************************************
 * 
 * Configuration && Define
 * 
 * ********************************************************************/
 
#define bool boolean

#define DELAY_MAIN_LOOP    		1000// 10 //µs, used at the end of each Loop(). Base Timer

#define DELAY_START_UP			1000 	//ms  time to wait at the end of setup(), to be sure the main board is ready !
#define DELAY_OFF				1000*30 // = 30sec
#define DELAY_INIT				100
/** delay for slow pin(default). Check if values changed and push then 
 * to the updateQueue **/
#define DELAY_CHECK_PIN_SLOW	1000 // *timer ( DELAY_MAIN_LOOP )
/** delay for fast pin. Check if values changed and push then 
 * to the updateQueue **/
#define DELAY_CHECK_PIN_FAST	10 // *timer ( DELAY_MAIN_LOOP )
/** psuh delay for analog input (use ADC) **/
#define DELAY_PROCESS_ANALOG	10000 // *timer ( DELAY_MAIN_LOOP )

/** we sum X time each analog value, to smooth result (average result) **/
#define SERIAL_BAUDRATE			115200

#define ANALOG_SUM				5

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
void setup_analog_timer()
{
    TCCR0A = 0;  // Setup analog interrupt
    OCR0B = 64;
    TIMSK0 |= (1<<OCIE0B);
}

void setAllPinInput()
{
	Serial.print(" > INIT INPUT TYPE < ");
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
    Serial.begin( SERIAL_BAUDRATE );  
    Serial.print(" START ID : ");
    Serial.print(BOARD_ID);
    setup_slave_master( ); 
  //  pinMode(3,OUTPUT);
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
	eps_manage();
	
    if ( board.check_state == BOARD_OFF )
    {
        Serial.print(" OFF ");
        delay( DELAY_OFF );
    }
    else if ( board.check_state == BOARD_W8_MASTER )
    {
		Serial.print(" W8 ");
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
		//delayMicroseconds( DELAY_MAIN_LOOP ); //__asm__("nop\n\t"); 
	}
}


/**
This timer is called 3906 timer per second. It is used to read Analog Input (using ADC)
*/
ISR( TIMER0_COMPB_vect )
{
/*	if ( need_check )
	{
		current_analog++;//next value to conv.
		if ( current_analog > 15 ) // number of ANALOG pin
		{
			current_analog = 0;
		}
		if ( IS_ANALOG(board.pin_values[current_analog]->type) )
		{
			#if defined(ADCSRB) && defined(MUX5)
			if(current_analog & 8)  // Reading channel 0-7 or 8-15?
				ADCSRB |= _BV(MUX5);
			else
				ADCSRB &= ~_BV(MUX5);
			#endif
			ADMUX = (ADMUX & ~(0x1F)) | (current_analog & 7);
			ADCSRA |= _BV(ADSC);  // start next conversion
			need_check = false;
		}	
	}
	else
	{
		if( (ADCSRA & _BV(ADSC)) == 0 )  // Conversion finished ?
		{
			board.pin_values[current_analog]->value = ADCW;
			need_check = true;
		}
	}*/
}

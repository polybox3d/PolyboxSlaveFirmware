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

#ifndef __PIN_H__
#define __PIN_H__

/***********************************************************************
 * ATMEGA 2560
 * 
 * Size of struct :
 * uint8_t : 8bits
 * int : 16bits
 * class Pin is => 8+16 => 24bits => 3 Bytes
 * 
 * A Pin can be OUT or INPUT
 *  INPUT pin are monitored and sent to master periodicaly when values
 * changed. Futhermore, user can specify FAST or STANDARD (default) 
 * monitoring. COUNTER pin is inherited from PIN, and provide a counter.
 * Check doc/code for futher information.
 * 
 * ********************************************************************/
 

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
 

/***********************************************************************
 * 
 * Defines
 * 
 * ********************************************************************/
 
#define PIN_TYPE_INPUT      INPUT //0
#define PIN_TYPE_OUTPUT     OUTPUT //1
#define PIN_TYPE_DIGITAL    0
#define PIN_TYPE_ANALOG     2
#define PIN_TYPE_DISABLE    4

#define PIN_TYPE_FAST_CHECK 8
#define PIN_TYPE_SLOW_CHECK 0

#define PIN_TYPE_COUNTER	16 
#define PIN_TYPE_STANDARD	0

#define PIN_NOONE           NULL


/***********************************************************************
 * 
 * Macros
 * 
 * ********************************************************************/

#define PIN_TYPE_IO_MASK   ( PIN_TYPE_INPUT | PIN_TYPE_OUTPUT )
#define PIN_TYPE_AD_MASK   ( PIN_TYPE_DIGITAL | PIN_TYPE_ANALOG )

#define IS_DIGITAL(n) ( (n & PIN_TYPE_AD_MASK) | ( n & PIN_TYPE_DISABLE)  ) == PIN_TYPE_DIGITAL
#define IS_ANALOG(n) ( (n & PIN_TYPE_AD_MASK) | ( n & PIN_TYPE_DISABLE)  ) ==  PIN_TYPE_ANALOG

#define IS_STANDARD(n) ( ( n & PIN_TYPE_COUNTER ) | ( n & PIN_TYPE_DISABLE)  ) ==  PIN_TYPE_STANDARD
#define IS_COUNTER(n) ( (n & PIN_TYPE_COUNTER) | ( n & PIN_TYPE_DISABLE)  ) ==  PIN_TYPE_COUNTER

#define GET_IO_TYPE type & PIN_TYPE_IO_MASK


/***********************************************************************
 * 
 * Class definition
 * 
 * ********************************************************************/
class Pin
{
	public :
	
	uint8_t type;
    int value;
    
	Pin(){
		type = PIN_TYPE_DISABLE;
		value = 0;
		};
//	virtual ~Pin(){};
		
};
#endif

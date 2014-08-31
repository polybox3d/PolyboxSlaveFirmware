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
 * @file Board.h
 * @author Florian Boudinet <florian.boudinet@gmail.com>
 * 
 * This file define Board class. The class store pin value,
 * type and usual board's function, like keep-alive or check update.
 * 
 * It's a part of Polybox-Firmware.
 * 
 * *********************************************************/

#ifndef __BOARD_H__
#define __BOARD_H__

#include <inttypes.h>
#include "Pin.h"
#include "CounterPin.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "QueueList.h"
#include "Update.h"
#include "eps.h"


/***********************************************************************
 * 
 * Defines
 * 
 * ********************************************************************/
 
#define BOARD_WAIT_VERSION				0
#define BOARD_WAIT_INIT					1
#define BOARD_WAIT_PONG					2
#define BOARD_OK						3
#define BOARD_BAD_VERSION				4
#define BOARD_OFF						5
#define BOARD_INIT						6
#define BOARD_W8_MASTER					7

#define BOARD_CONNECTED_CHECK_DELAY     10 //total time is : 100ms * BOARD_CONNECTED_CHECK_DELAY = 1sec
#define BOARD_UPDATE_CHECK_DELAY        1 //total time is : 100ms * BOARD_UPDATE_CHECK_DELAY = 100mssec
#define BOARD_SEND_GET_DELAY            10*60 // 1min

#define ANALOG_TOLERANCE_RANGE			2 // 0.0049V per unit so it's a window of +/- 0.0098V 

/***********************************************************************
 * 
 * Class definition
 * 
 * ********************************************************************/
class Board
{
    public:
    
    Board();
    ~Board();
    /**
     * Watch pins's state, and adds new values ​​to the table pin_value.
     * */
    void check_pins_update(uint8_t type = 0);
    /**
     * Process and compute analogic values.
     * **/
    void process_analog();
   /**
    * Initialize pin_value table.
    * **/
    void init_pin_table(); 
    
 /***********************************************************************
 * Define Read/Write function for virtual pin.
 * ********************************************************************/
    //READ
    int read_bpin( uint8_t pin );
    uint8_t read_bpin_type( uint8_t pin );
    //WRITE
    int write_bpin( uint8_t pin, int value );
    uint8_t write_bpin_type( uint8_t pin, uint8_t type );
    
 /***********************************************************************
 * Attributes
 * ********************************************************************/
	/**
	 * Store pin value of the Board.
	 * **/
    Pin* pin_values[PINS_PER_BOARD];
    
    /**
     * Static variable representing the next Board ID (for I2C). 
     * **/
    static uint8_t next_id;
    
    uint8_t connected;
    
    uint8_t check_state;
    
    /**
     * I2C ID of this Board. It's defined by using next_id static variable.
     * **/
    uint8_t i2c_id;
    
    /**
     * List of modified pin values ​​to send to the master or slave.
     * **/
    QueueList<Update> pin_update_queue;
   
};




#endif

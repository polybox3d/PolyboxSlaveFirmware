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

#ifndef __eps_H__
#define __eps_H__

/***********************************************************************
 * EPS : Extension Pin System
 * 
 * Provide more pin for a arduino master. You can use as much Arduino 
 * slave you like.
 * 
 * Slave doesnt know his pin-mapping, and all the configuration is 
 * provided by the Master. A time_out system detect when a slave/master
 *  is deconnected.
 * 
 *  A Pin can be OUT or INPUT.
 * 
 *  INPUT pin are monitored and sent to master periodicaly when values
 * changed. Futhermore, user can specify FAST or STANDARD (default) 
 * monitoring. COUNTER pin is inherited from PIN, and provide a counter.
 * Check doc/code for futher information.
 * 
 * Handshake :
 * 
 * Wait > Check Version > Init > Running
 * 
 * ********************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#define I2C_WRITE write 
#define I2C_READ read
#include "Arduino.h"
#else
#include "WProgram.h"
#define I2C_WRITE send
#define I2C_READ receive
#endif

#include <inttypes.h>


#include "Configuration_eps.h"
#include "Board.h"
#include <Wire.h>


/***********************************************************************
 * 
 * Defines
 * 
 * ********************************************************************/

#define EPS_PROTOCOL_VERSION 1
#define EPS_MIN_VERSION_REQUIRE   1

#define EPS_SETUP           1
#define EPS_SET             2
#define EPS_GET             3
#define EPS_ALL             4
#define EPS_PING            5
#define EPS_PONG            6
#define EPS_RESET           7
#define EPS_VERSION         8
#define EPS_INIT            9
#define EPS_TOKEN			10
#define EPS_ACK				31

#define MASTER_ID           1

#define WRITE_VPIN( p, v) eps_set_vpin_value( p, v)
#define READ_VPIN( p ) eps_read_vpin_value( p )
#define READ_VPIN_MODE( p ) eps_read_vpin_type( p )
#define VPIN_MODE( p, v) eps_write_vpin_type( p, v)

/***********************************************************************
 * 
 * Extern Variables
 * 
 * ********************************************************************/

class Board;

extern Board boards[NUM_BOARD];
extern bool send_entries_flag;
extern volatile bool has_token;
extern uint32_t i2c_ack_resend_count;
extern uint32_t i2c_timer;

/***********************************************************************
 * 
 * Read/Write Functions
 * 
 * ********************************************************************/

uint8_t vpin2bpin(int vpin);
uint8_t vpin2board(int vpin);
// READ
int board_read_bpin_value( uint8_t b, uint8_t pin );
uint8_t board_read_bpin_type( uint8_t b, uint8_t pin );

int eps_read_vpin_value( int pin );
uint8_t eps_read_vpin_type( int pin );

// WRITE
void eps_set_vpin_value( int pin, int value);
void eps_write_vpin_type( int pin, uint8_t type);


/***********************************************************************
 * 
 * Functions
 * 
 * ********************************************************************/

void eps_send_board_update(uint8_t dest);
byte eps_send_entries();

void eps_manage();

void i2cRequestEvent();
void i2cReceiveEvent(int howMany);
void setup_slave_master();
void printAllPin();


#ifdef IS_MASTER // glitch
 void operator delete(void * p);
 void * operator new(size_t size); 
#endif


#endif

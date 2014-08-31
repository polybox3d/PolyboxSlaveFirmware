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

/***********************************************************************
 * Configuration file.
 * Variable you can adjust, like delay timer.
 * 
 **********************************************************************/
#ifndef __CONFIGURATION_EPS_H__
#define __CONFIGURATION_EPS_H__

#ifndef SLAVE_ID
  #error "You must define a slave number. You can do it though make command or using SLAVE_ID define"
#endif
#if SLAVE_ID == -1
  #error "You must define a board number. You can do it though make command or using SLAVE_ID define"
#endif

#define IS_SLAVE			1          
//#define IS_MASTER			1

#ifdef IS_SLAVE
 #define BOARD_ID (SLAVE_ID + 1)
 #define board boards[0]
// #define WRITE_PIN(p,v) boards[0].write_bpin(p,v)   <--- / NOOB !
// #define PIN_MODE(p,t) boards[0].write_bpin_type(p,t)
#endif


#define NUM_BOARD			1
#define PINS_PER_BOARD		(54+16) // Numeric. + Analog.
#define PIN_ANALOG			54

#define EPS_BASETIME		10
#define HANDSHAKE_TIME		(1000/EPS_BASETIME)
#define PING_PONG_DELAY		(10000/EPS_BASETIME)
#define GET_DELAY			(100000/EPS_BASETIME)
#define SEND_UPDATE_DELAY	(1000/EPS_BASETIME)

#endif

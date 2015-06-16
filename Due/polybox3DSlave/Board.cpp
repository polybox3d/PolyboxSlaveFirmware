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
#include "Board.h"

uint8_t Board::next_id = 1;

Board::Board()
{
	for (uint8_t i = 0; i < PINS_PER_BOARD ; ++i )
	{
			pin_values[i] = new Pin();
	}
	
    this->connected = false;
    this->check_state = BOARD_W8_MASTER;
    this->i2c_id = next_id;
    next_id++;
}
Board::~Board()
{
// delete Pin()
}

void Board::process_analog()
{
    int value=0;
    int board_value = 0;
    // we process only analog compatible output pin (i.e pwm out). Check Configuration_eps.h for value/range
    for ( uint8_t i = PIN_ANALOG; i < PINS_PER_BOARD ; ++i )
    {
        // we process only INPUT type pin. Output are controled by master.
        if ( (pin_values[i]->GET_IO_TYPE) == PIN_TYPE_INPUT && IS_ANALOG( pin_values[i]->type ) )
        {
			value = analogRead( i-PIN_ANALOG );
			board_value = pin_values[i]->value ;
			if ( value > ( board_value + ANALOG_TOLERANCE_RANGE ) || value < ( board_value - ANALOG_TOLERANCE_RANGE ) )
			{
				write_bpin( i, value );
				pin_update_queue.push( Update{ i, EPS_SET } );				
			}
		}
	}
}

void Board::check_pins_update(uint8_t type)
{
    int value=0;
    for ( uint8_t i = 0; i < PINS_PER_BOARD ; ++i )
    {
        // we process only INPUT type pin. Output are controled by master.
        if ( (pin_values[i]->GET_IO_TYPE) == PIN_TYPE_INPUT )
        {
			// Standard pin (type=0)
			if ( IS_STANDARD(pin_values[i]->type)  &&  ( ( pin_values[i]->type & PIN_TYPE_FAST_CHECK ) == type) )
			{
				int board_value = read_bpin( i );
				if ( IS_DIGITAL( pin_values[i]->type ) )
				{
					value = digitalRead( i );
					if ( board_value != value )
					{
						/* Write the value inside the table */
						write_bpin( i, value );
						/* We push the Update pin into the pin table for sending through I2C */
						pin_update_queue.push( Update{ i, EPS_SET } );
					}
				}
			}
			// check Counter type
			else if ( IS_COUNTER(pin_values[i]->type)  )
			{
				// update counter value if FAST check
				CounterPin *c = static_cast<CounterPin*>(pin_values[i]);
				/* If the current state checked is FAST_PIN, we increase the current pin value.
				 * Else (so STANDARD_PIN (type=0), we send update of fast pin if needed. */
				if (  (type == PIN_TYPE_FAST_CHECK) )
				{
					value = digitalRead( i );
					if ( value != c->lastState )
					{
						c->lastState =  !c->lastState;
						c->updated = true; // set updated to true, so firmware will queue data
						write_bpin( i, read_bpin( i )+1 );
					}
				}
				// push updatequeue, so counter can be sent to master when NORMAL_CHECK
				else if ( c->updated )
				{
					c->updated = false;
					pin_update_queue.push( Update{ i, EPS_SET } );
				}
			}
		}
		else
		{
		}
    }
}
void Board::init_pin_table()
{
    for( int i =0; i<PINS_PER_BOARD ; ++i ) // Loop to send all entries
    {
        write_bpin_type( i, PIN_TYPE_INPUT );
        write_bpin( i, 0 );
    }
}
// READ
int Board::read_bpin( uint8_t pin )
{
    return pin_values[pin]->value;
}
uint8_t Board::read_bpin_type( uint8_t pin )
{
    return pin_values[pin]->type;
}
// WRITE
/** Write Board pin. Write into the current pin table given value.**/
int Board::write_bpin(  uint8_t pin, int value )
{
    pin_values[pin]->value = value;
}

uint8_t Board::write_bpin_type( uint8_t pin, uint8_t type )
{
    pin_values[pin]->type = type;
}

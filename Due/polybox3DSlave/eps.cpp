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

#include "eps.h"

Board boards[NUM_BOARD];
bool send_entries_flag = false;
int next_send_pin=0;
volatile uint16_t restart_counter =0 ;
volatile bool has_token = false ;

///< ACK system
bool send_ack = false;
uint8_t i2c_destination = 0;
uint8_t i2c_current_buffer_size = 0;
uint8_t i2c_buffer[BUFFER_LENGTH];
uint32_t i2c_ack_resend_count = 0;
uint32_t i2c_timer = 0;
uint8_t i2c_ack_reply_destination = 1;
uint8_t need_restart_handshake = false;

void(* reset_board) (void) = 0;

void setup_slave_master(  ) {
    Wire.begin( BOARD_ID );          // join i2c bus
    Wire.onReceive(i2cReceiveEvent); // register event
    Wire.onRequest(i2cRequestEvent); // register event
}

void eps_manage()
{
    i2c_timer++;
}
uint8_t vpin2bpin(int vpin)
{
    return vpin%PINS_PER_BOARD;
}
 uint8_t vpin2board(int vpin)
{
    return vpin/PINS_PER_BOARD;
}
void printAllPin()
{
    #if OUTPUT_SERIAL
    for ( uint8_t i =0 ; i<NUM_BOARD ; ++i )
    {

        Serial.print("Board");
        Serial.println(i);
        for ( uint8_t h = 0 ; h<PINS_PER_BOARD ; ++h )
        {
            Serial.print(h);
            Serial.print(":");
            Serial.print( boards[i].pin_values[h]->value);
            Serial.print("  ");
        }
    }
    #endif
}
// READ
int eps_read_vpin_value( int pin )
{
    return boards[vpin2board(pin)].read_bpin( vpin2bpin(pin) );
}
 uint8_t eps_read_vpin_type( int pin )
{
    return boards[vpin2board(pin)].read_bpin_type( vpin2bpin(pin) );
}

int board_read_bpin_value( uint8_t b, uint8_t pin )
{
    return boards[b].read_bpin( pin );
}
 uint8_t board_read_bpin_type( uint8_t b, uint8_t pin )
{
    return boards[b].read_bpin_type( pin );
}

// WRITE

void eps_set_vpin_value( int pin, uint16_t value) {
    board.write_bpin( pin, value );

    if ( pin >= PINS_PER_BOARD ) // start virtual pin (i.e other Arduino board)
    {
        return;
    }
    else
    {
        /*if ( IS_DIGITAL(boards[board_n].pin_values[real_pin]->type) ) // digital
        {
            digitalWrite( real_pin, value );
        }
        else
        {*/
        #if OUTPUT_SERIAL
            if ( pin == 0 || pin == 1)
                return;
        #endif
     //   digitalWrite(pin, value);
        analogWrite(pin, value);

    }
}

void eps_write_vpin_type( int pin, uint8_t type) {

    if ( pin >= PINS_PER_BOARD ) // start virtual pin (i.e other Arduino board)
    // So we need to update the fifo queue.
    {
        return;
    }
    else
    {
        //CORRECT_ANALOG_PINMODE_BUG
// @todo
        pinMode(pin, (type & PIN_TYPE_IO_MASK) );
    }
    board.pin_values[pin]->type = type ;
}

void eps_send_board_update(uint8_t dest)
{
    Update up;
    byte count=0;
    uint16_t value=0;
    count = 0;
    uint8_t buffer[BUFFER_LENGTH];
    while ( !boards[dest].pin_update_queue.isEmpty() && count < (BUFFER_LENGTH-5))
    {
        up = boards[dest].pin_update_queue.pop();
        buffer[count++] = up.type;
        buffer[count++] = up.pin;
        if ( up.type == EPS_SET ) {
            value = boards[dest].read_bpin(up.pin);
            buffer[count++] = (byte) ( ( value & 0xFF00)>>8 );
            buffer[count++] = (byte) ( value & 0x00FF );
        }
        else if ( up.type == EPS_SETUP )
        {
            buffer[count++] = boards[dest].read_bpin_type(up.pin);
        }
    }
    while ( count < BUFFER_LENGTH )
    {
        buffer[count++] = EPS_SET_END;
    }
    Wire.I2C_WRITE( buffer, count );
}


byte eps_send_entries()
{
    uint8_t j;
    for( j =0; j< ((BUFFER_LENGTH)/3) ; ++j ) // Loop to send all entries. TWI buffer is only 32 Byte. We send 3 byte during each "for" loop
    {
        if ( ( boards[0].read_bpin_type(next_send_pin)  & PIN_TYPE_IO_MASK ) == PIN_TYPE_INPUT )
        {
            Wire.I2C_WRITE( EPS_SET );
            Wire.I2C_WRITE( next_send_pin );
            Wire.I2C_WRITE( (byte) ((boards[0].pin_values[next_send_pin]->value & 0xFF00) >> 8)  );
            Wire.I2C_WRITE( (byte) (boards[0].pin_values[next_send_pin]->value & 0x00FF) );
        }
        next_send_pin = (next_send_pin+1) % PINS_PER_BOARD;
        if ( next_send_pin == 0 )
        {
            return false;
        }
    }
    return true;
}

void i2cRequestEvent()
{
    uint8_t action = board.check_state;
    if ( need_restart_handshake )
    {
        need_restart_handshake = false;
        Wire.I2C_WRITE( EPS_RESET );
        #if OUTPUT_SERIAL
            Serial.print(" RESHAKE MASTER SIDE ");
        #endif
    }
    else if ( action == BOARD_WAIT_PONG && board.connected )
    {
        Wire.I2C_WRITE( EPS_PONG );
        board.check_state = BOARD_OK;
        #if OUTPUT_SERIAL
            Serial.print(".");
        #endif
    }
    else if ( send_entries_flag && board.connected )
    {
        eps_send_entries();
        send_entries_flag = false;
        #if OUTPUT_SERIAL
            Serial.print("+");
        #endif
    }
    else if ( action == EPS_RESET )
    {
        #if OUTPUT_SERIAL
            Serial.print(" RESET BOARD Q ");
        #endif
        //reset_board();
    }
    else if ( action == BOARD_WAIT_INIT )
    {
        Wire.I2C_WRITE( EPS_INIT );
        board.check_state = BOARD_OK;
        board.connected = true;
        #if OUTPUT_SERIAL
            Serial.print(" INITQ ");
        #endif
    }
    else if ( has_token )
    {
        eps_send_board_update(0);
        has_token = false;
        #if OUTPUT_SERIAL
            Serial.print("*");
        #endif
    }
    else if ( action == BOARD_WAIT_VERSION )
    {
        Wire.I2C_WRITE( EPS_PROTOCOL_VERSION );
        board.check_state = BOARD_W8_MASTER;
        #if OUTPUT_SERIAL
            Serial.print(" VERSQ ");
        #endif
    }
    else
    {
        #if OUTPUT_SERIAL
            Serial.print(" elseREQ? ");
            Serial.print( action );
        #endif
    }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void i2cReceiveEvent(int howMany)
{
    byte pin = 0;
    int value = 0;
    int sender = Wire.I2C_READ() -1;
    byte action = Wire.read();

    ///< ACK system
    i2c_ack_reply_destination = sender + 1 ;
    send_ack = true;

    if ( action == EPS_GET && board.connected )
    {
        send_entries_flag = true;
    }
    else if ( action == EPS_TOKEN && board.connected)
    {
        has_token = true;
    }
    else if ( action == EPS_TOKEN && ! board.connected)
    {
        need_restart_handshake = true;
    }
    else if ( action == EPS_SET  && board.connected)
    {
        if ( Wire.available() )
        {
            pin = Wire.I2C_READ();
            value = Wire.I2C_READ()<<8;
            value += Wire.I2C_READ();
            #if OUTPUT_SERIAL
            if ( pin != 0 && pin != 1)
            #endif
            {
                WRITE_VPIN( pin, value );
                #if OUTPUT_SERIAL
                    Serial.print(" SET ");
                    Serial.print(pin);
                    Serial.print(":");
                    Serial.print(value);
                #endif
            }
        }
    }
    else if ( action == EPS_SETUP && board.connected )
    {
        while ( Wire.available() )
        {
            pin = Wire.I2C_READ();
            value = Wire.I2C_READ();
            #if OUTPUT_SERIAL
            if ( pin != 0 && pin != 1)
            #endif
            {
                VPIN_MODE( pin, value );
                #if OUTPUT_SERIAL
                    Serial.print(" SETUP ");
                    Serial.print(pin);
                    Serial.print(":");
                    Serial.print(value);
                #endif
            }
        }
    }
    else if ( action == EPS_ALL && board.connected)
    {
        #if OUTPUT_SERIAL
            Serial.print(" ***ALL/UP*** ");
        #endif
        while ( Wire.available() )
        {
            action = Wire.I2C_READ();
            pin = Wire.I2C_READ();
            value=0;

            if ( action == EPS_SET)
            {
                value = Wire.I2C_READ() << 8;
                value += Wire.I2C_READ();
                #if OUTPUT_SERIAL
                if ( pin != 0 && pin != 1)
                #endif
                {
                    WRITE_VPIN( pin, value );
                    #if OUTPUT_SERIAL
                        Serial.print(" SET_ ");
                        Serial.print(pin);
                        Serial.print(":");
                        Serial.print(value);
                    #endif
                }
            }
            else if ( action == EPS_SETUP)
            {
                value = Wire.I2C_READ();
                #if OUTPUT_SERIAL
                if ( pin != 0 && pin != 1)
                #endif
                {
                    VPIN_MODE( pin, value );
                    #if OUTPUT_SERIAL
                        Serial.print(" SETUP_ ");
                        Serial.print(pin);
                        Serial.print(":");
                        Serial.print(value);
                    #endif
                }
            }
        }

    }
    else if ( action == EPS_RESET )
    {
        #if OUTPUT_SERIAL
            Serial.print(" RESET BOARD ");
        #endif
        //reset_board();
    }
    else if ( action == EPS_INIT )
    {
        board.check_state = BOARD_WAIT_INIT;
    }
    else if ( action == EPS_VERSION )
    {
        restart_counter++;
        board.connected = false;
        if ( Wire.available() )
        {
            value = Wire.read();
            if ( value > EPS_PROTOCOL_VERSION )
            {
                board.check_state = BOARD_BAD_VERSION;
            }
            else
            {
                board.check_state = BOARD_WAIT_VERSION;
            }
        }
    }
    else if ( action == EPS_PING && board.connected )
    {
        board.check_state = BOARD_WAIT_PONG;
    }
    else
    {
        #if OUTPUT_SERIAL
            Serial.print(" else? ");
            Serial.print( action );
        #endif
    }
}


#ifdef IS_MASTER
void operator delete(void * p) // or delete(void *, std::size_t)
{
  free(p);
}
void * operator new(size_t size)
{
  return malloc(size);
}
#endif

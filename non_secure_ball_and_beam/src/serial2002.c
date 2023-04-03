/* Serial2002 - a lightweight Serial2002 implementation.
 * Copyright (C) Anders Blomdell Department of Automatic Control
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Modified for the nRF5340 platform by Martin Gunnarsson and Nils Vreman.
 */

#include "serial2002.h"
#include <stdio.h>
#include <string.h>
#include "drivers/uart.h"

// UART driver
extern const struct device *serial2002_uart_dev;

//write directly to meas_raw
extern uint32_t raw_meas[2];

/* Internal SerialIO variables */
static volatile unsigned long serialio_value = 0;
static volatile unsigned char serialio_channel = 255;
static volatile unsigned char serialio_length = 0;

static unsigned long read_value = 0;
static unsigned char read_channel = 255;

int32_t uart_status = 0;
uint32_t received_bytes = 0;

/* Send byte over UART to the plant */
static inline void uart_send(unsigned char byte, unsigned long len)
{
    uart_poll_out(serial2002_uart_dev, byte);
}


/* Send one bit to specific channel */
void serialio_putbit( unsigned char channel, unsigned char value)
{
    if (value) {
        uart_send( 0x20 | (channel & 0x1f), 1);
    } else {
        uart_send( 0x00 | (channel & 0x1f), 1);
    }
}

/* Poll specific channel. TODO: Is this correct? */
void serialio_getchannel( unsigned char channel)
{
    uint8_t byte = 0x60 | (channel & 0x1f);
    uart_send( byte, 1 );
}

/* Send value (less than 4 bytes) to channel */
void serialio_putchannel( unsigned char channel, unsigned long value )
{
    if (value >= (1L<<30)) { uart_send( 0x80 | ((value >> 30) & 0x03), 1); }
    if (value >= (1L<<23)) { uart_send( 0x80 | ((value >> 23) & 0x7f), 1); }
    if (value >= (1L<<16)) { uart_send( 0x80 | ((value >> 16) & 0x7f), 1); }
    if (value >= (1L<< 9)) { uart_send( 0x80 | ((value >> 9) & 0x7f), 1); }
    uart_send(0x80 | ((value >> 2) & 0x7f), 1);
    uart_send( ((value << 5) & 0x60) | (channel & 0x1f), 1);
}

/* Reads channel and value from character ch */
serialio_rxc_status serialio_RXC(unsigned char ch, unsigned long *value, unsigned char *channel)
{
    unsigned char result = serialio_error;

    if (serialio_length == 0) {
        serialio_value = 0;
    }
    serialio_length++;

    /* check whether channel has first bit set */
    if ((ch & 0x80) == 0x80) {
        // Collect yet another byte for later processing
        serialio_value = (serialio_value << 7) | (ch & 0x7f);
        result = serialio_more;
    } else {
        serialio_value = (serialio_value << 2) | ((ch & 0x60) >> 5);
        serialio_channel = ch & 0x1f;
        if (serialio_length == 1) {
            switch (serialio_value & 0x03) {
                // Digital output buffer (ULN2803A) is inverting
                case 0: { result = serialio_clearbit; } break;
                case 1: { result = serialio_setbit; } break;
                case 2: { result = serialio_pollbit; } break;
                case 3: { result = serialio_pollchannel; } break;
            }
        } else {
            result = serialio_setchannel;
        }
        serialio_length = 0;
    }
    *value = serialio_value;
    *channel = serialio_channel;
    return result;
}

void receive_value()
{
    /* Initialise variables */
    serialio_rxc_status status;
    uint8_t c;
    uint8_t more = 1;

    /* While we have more data to read */
    while ( more ) {

        /* Read one byte from UART. uart_poll_in() return 0 on success, otherwise negative number */
         
        uart_status = uart_poll_in(serial2002_uart_dev, &c);
        /* If we read something */
        if ( uart_status == 0 ) {

            /* Increment the number of received bytes */
            received_bytes += uart_status;

            /* Read 32 Bit value and channel from character c */
            status = serialio_RXC(c, &read_value, &read_channel);

            /* Depending on status, store the read value in raw_meas array */
            if ( status == serialio_setchannel ) {

                switch ( read_channel ) { 
                    case BEAM_ANG_CHANNEL:
                        raw_meas[0] = read_value;
                        break;
                    case BALL_POS_CHANNEL:
                        raw_meas[1] = read_value;
                        break;
                    default:
                        break;
                }
                more = 0;
                return;

            } else if ( status == serialio_pollchannel ) { /* NOTE: Shouldn't occur */ 
                more = 0;
            } else {                                       /* or continue reading from serialio */
                more = 1;   
            }

        } else if (uart_status == -1) {
        //    printf("Reading from empty buffer\n");
        } else {
       //    printf("Read fail %02x  \n", c);
            uart_status = status;
            return;
        }
    }
}


int32_t get_pos(void)
{
    serialio_getchannel(BALL_POS_CHANNEL);
    return 0;
}

int32_t get_ang(void)
{
    serialio_getchannel(BEAM_ANG_CHANNEL);
    return 0;
}

void set_ctrl(float u)
{
    /* convert control signal (in range -10 to 10) to uint32_t with 16 bit
     * resolution */
    uint32_t u_raw;
    if (u > 10.0f)
        u_raw = 0xFFFF; // 0xFFFF = 10 V = FULL TORQUE ONE DIRECTION (hex, control representation, ...)
    else if (u < -10.0f)
        u_raw = 0x0000; // 0x0000 = -10 V = FULL TORQUE OTHER DIRECTION (hex, control representation, ...)
    else
        u_raw = (uint32_t) ((u + 10.0f) / 20.0f * (float) 0xFFFF);

    // Actuate (channel 0) with u_raw (in a 16 bit representation)
    serialio_putchannel(0x00, u_raw);  // Velocity represented by 16 bits in AVR code
}

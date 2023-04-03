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

/*
 * Digital in/out and poll commands are sent as one byte:
 *
 *   +-+-+-+-+-+-+-+-+
 *   |0|0 0|  chan   | Bit clear
 *   +-+-+-+-+-+-+-+-+
 *
 *   +-+-+-+-+-+-+-+-+
 *   |0|0 1|  chan   | Bit set
 *   +-+-+-+-+-+-+-+-+
 *
 *   +-+-+-+-+-+-+-+-+
 *   |0|1 0|  chan   | Bit get
 *   +-+-+-+-+-+-+-+-+
 *
 *   +-+-+-+-+-+-+-+-+
 *   |0|1 1|  chan   | Channel get
 *   +-+-+-+-+-+-+-+-+
 *
 *
 * Channels are sent as 2 to 6 bytes, depending on resolution:
 *
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+ 
 * 2 |1| bit8...bit2 |  |0|bit|  chan   |
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+ 
 *
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+ 
 * 3 |1|bit15...bit9 |  |1| bit8...bit2 |  |0|bit|  chan   |
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+ 
 *
 *   ...
 *
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+     +-+-+-+-+-+-+-+-+ 
 * 6 |1|bit31...bit30|  |1|bit29...bit23| ... |0|bit|  chan   |
 *   +-+-+-+-+-+-+-+-+  +-+-+-+-+-+-+-+-+     +-+-+-+-+-+-+-+-+ 
 *
 *
 *
 * Channel 31 is special, as it serves as the configuration channel. When
 * reading from it multiple responses are sent with the following layout
 *
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * |           command specific data           |cmd|kind |conf chan|
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 *
 *  kind: 000 == end of configuration
 *        001 == digital in
 *        010 == digital out
 *        011 == analog in
 *        100 == analog out
 *        101 == counter in
 *
 *cmd == 0 (Resolution)
 *
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * |                               | # of bits |0 0|kind |conf chan|
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 *
 *  # of bits (1..32) 
 *
 *cmd == 1 (Minimum value)
 *
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * |              minimum              |S| unit|0 1|kind |conf chan|
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * 
 *  S (sign): 0 == +
 *            1 == -
 *  unit: 000 == V
 *        001 == mV
 *        010 == uV
 *        100 == A
 *
 *cmd == 2 (Maximum value)
 *
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * |              maximum              |S| unit|1 0|kind |conf chan|
 * +-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-|-+-+-+-+-+-+-+-+
 * 
 *  S (sign): 0 == +
 *            1 == -
 *  unit: 000 == V
 *        001 == mV
 *        010 == uV
 *        100 == A
 */

#include <stdint.h>

typedef enum { 
  serialio_error, serialio_more, serialio_clearbit, serialio_setbit, 
  serialio_setchannel, serialio_pollbit, serialio_pollchannel 
} serialio_rxc_status;

void serialio_putbit( unsigned char channel, unsigned char value);

void serialio_getchannel( unsigned char channel);


void serialio_putchannel( unsigned char channel, 
				       unsigned long value);

serialio_rxc_status serialio_RXC(unsigned char ch, unsigned long *value, unsigned char *channel);

//TODO return error value here
void receive_value();

int32_t get_pos(); 
int32_t get_ang(); 
void set_ctrl(float);

#define BEAM_ANG_CHANNEL         0x00
#define BALL_POS_CHANNEL         0x01

#define CONF_DIG_IN(channel) (0x20 | (channel)&0x1f)
#define CONF_DIG_OUT(channel) (0x40 | (channel)&0x1f)

#define CONF_END() serialio_putchannel(31, 0)
#define CONF_DIGITAL_IN(chan, config) \
  serialio_putchannel(31, (0x20|(chan&0x1f)|(config&0xffffff00)))
#define CONF_DIGITAL_OUT(chan, config) \
  serialio_putchannel(31, (0x40|(chan&0x1f)|(config&0xffffff00)))
#define CONF_ANALOG_IN(chan, config) \
  serialio_putchannel(31, (0x60|(chan&0x1f)|(config&0xffffff00)))
#define CONF_ANALOG_OUT(chan, config) \
  serialio_putchannel(31, (0x80|(chan&0x1f)|(config&0xffffff00)))
#define CONF_ENCODER_IN(chan, config) \
  serialio_putchannel(31, (0xa0|(chan&0x1f)|(config&0xffffff00)))
#define CONF_RESOLUTION(bits) (((bits)<<10)|0x000)
#define CONF_MIN(value) ((value)|0x100)
#define CONF_MAX(value) ((value)|0x200)
#define CONF_NEGATIVE_VOLT(volt) (((long)(volt)<<14)|0x2000)
#define CONF_POSITIVE_VOLT(volt) ((long)(volt)<<14)
#define CONF_NEGATIVE_MILLIVOLT(millivolt) (((long)(millivolt)<<14)|0x2400)
#define CONF_POSITIVE_MILLIVOLT(millivolt) ((long)(millivolt)<<14|0x400)
#define CONF_POSITIVE_AMPERE(ampere) (((long)(ampere)<<14)|0x1000)



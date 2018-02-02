/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FRSKY_COMMON_H_
#define FRSKY_COMMON_H_

#include <stdbool.h>
#include <stdint.h>

#include "iface_cc2500.h"

extern uint8_t packet[40];
extern uint16_t state;
extern uint8_t packet_count;
extern uint8_t option, prev_option;

#define FRSKY_HOPTABLE_SIZE 47
extern uint8_t cal_data[FRSKY_HOPTABLE_SIZE];

// Telemetry
#define MAX_PKT 29
extern uint8_t pkt[MAX_PKT];

enum {
	FRSKY_BIND		= 0,
	FRSKY_BIND_DONE	= 1000,
	FRSKY_DATA1,
	FRSKY_DATA2,
	FRSKY_DATA3,
	FRSKY_DATA4,
	FRSKY_DATA5
};

//void Frsky_init_hop(void);
void FRSKY_init_cc2500(void);

void frsky_init(void);
uint8_t frsky_check_transceiver(void);
void frsky_init_timer(void);

#endif  // FRSKY_COMMON_H_

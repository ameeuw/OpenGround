/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#ifndef FRSKY_H_
#define FRSKY_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "cc2500.h"
#include "iface_cc2500.h"

#define FRSKY_HOPTABLE_SIZE 47
#define FRSKY_PACKET_LENGTH 17
#define FRSKY_PACKET_BUFFER_SIZE (FRSKY_PACKET_LENGTH+3)
#define FRSKY_COUNT_RXSTATS 20

void frsky_init(void);
uint8_t frsky_check_transceiver(void);
void frsky_configure(void);
void frsky_enter_bindmode(void);
void frsky_tx_set_enabled(uint32_t enabled);
void frsky_set_channel(uint8_t hop_index);
void frsky_init_timer(void);


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
    FRSKY_BIND      = 0,
    FRSKY_BIND_DONE = 1000,
    FRSKY_DATA1,
    FRSKY_DATA2,
    FRSKY_DATA3,
    FRSKY_DATA4,
    FRSKY_DATA5
};

// void Frsky_init_hop(void);
void FRSKY_init_cc2500(void);

#endif  // FRSKY_H_

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

#ifndef FRSKY_TX_CLONE_H_
#define FRSKY_TX_CLONE_H_

#include "frsky.h"

void frsky_do_clone_prepare(void);
void frsky_do_clone_finish(void);
void frsky_autotune_prepare(void);
uint32_t frsky_autotune_do(void);
void frsky_autotune_finish(void);
void frsky_fetch_txid_and_hoptable_prepare(void);
uint32_t frsky_fetch_txid_and_hoptable_do(void);
void frsky_fetch_txid_and_hoptable_finish(void);
void frsky_enter_rxmode(uint8_t);
void frsky_tune_channel(uint8_t);
void frsky_configure_address(void);
void frsky_handle_overflows(void);
void frsky_calib_pll(void);

#define FRSKY_VALID_FRAMELENGTH(_b) (_b[0] == 0x11)
#define FRSKY_VALID_CRC(_b)     (_b[19] & 0x80)
#define FRSKY_VALID_TXID(_b) ((_b[1] == storage.frsky_txid[0]) && (_b[2] == storage.frsky_txid[1])
#define FRSKY_VALID_PACKET_BIND(_b) \
    (FRSKY_VALID_FRAMELENGTH(_b) && FRSKY_VALID_CRC(_b) && (_b[2] == 0x01))
#define FRSKY_VALID_PACKET(_b) \
    (FRSKY_VALID_FRAMELENGTH(_b) && FRSKY_VALID_CRC(_b) && FRSKY_VALID_TXID(_b) )

#endif  // FRSKY_TX_CLONE_H_

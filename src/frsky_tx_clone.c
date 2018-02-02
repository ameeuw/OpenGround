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

#include "debug.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "frsky.h"
#include "debug.h"
#include "timeout.h"
#include "led.h"
#include "delay.h"
#include "wdt.h"
#include "cc2500.h"
#include "io.h"
#include "clocksource.h"
#include "storage.h"
#include "adc.h"
#include "telemetry.h"

#include "frsky_tx_clone.h"

#define FRSKY_DEBUG_BIND_DATA 0
#define FRSKY_DEBUG_HOPTABLE 1

// DONE when n times a one:
#define MAX_BIND_PACKET_COUNT 10
#define HOPDATA_RECEIVE_DONE ((1  <<  (MAX_BIND_PACKET_COUNT))-1)

// pll calibration
static uint8_t frsky_calib_fscal1_table[FRSKY_HOPTABLE_SIZE];
static uint8_t frsky_calib_fscal2;
static uint8_t frsky_calib_fscal3;

static int8_t frsky_fscal0_min;
static int8_t frsky_fscal0_max;
static uint16_t frsky_bind_packet_hoptable_flags;
static volatile uint32_t frsky_state;

static uint8_t frsky_bind_packet_received;
static volatile uint8_t frsky_packet_buffer[FRSKY_PACKET_BUFFER_SIZE];
static volatile uint8_t frsky_packet_received;

void frsky_configure(void) {
    debug("frsky: configure\n"); debug_flush();

    // start idle
    cc2500_strobe(RFST_SIDLE);

    // IOCFG0,1,2 is set in hal code(it is specific to the board used)
    cc2500_set_gdo_mode();

    // normal config
    cc2500_set_register(MCSM1    , 0x0F);  // go back to rx after transmission completed
    cc2500_set_register(MCSM0    , 0x18);
    cc2500_set_register(PKTLEN   , FRSKY_PACKET_LENGTH);  // on 251x this has to be exactly our size
    cc2500_set_register(PKTCTRL0 , 0x05);
    cc2500_set_register(PA_TABLE0, 0xFF);
    cc2500_set_register(FSCTRL1  , 0x08);  // D4R-II seems to set 0x68 here ?! instead of 0x08
    cc2500_set_register(FSCTRL0  , 0x00);
    // set base freq 2404 mhz
    cc2500_set_register(FREQ2    , 0x5C);
    cc2500_set_register(FREQ1    , 0x76);
    cc2500_set_register(FREQ0    , 0x27);
    cc2500_set_register(MDMCFG4  , 0xAA);
    cc2500_set_register(MDMCFG3  , 0x39);
    cc2500_set_register(MDMCFG2  , 0x11);
    cc2500_set_register(MDMCFG1  , 0x23);
    cc2500_set_register(MDMCFG0  , 0x7A);
    cc2500_set_register(DEVIATN  , 0x42);
    cc2500_set_register(FOCCFG   , 0x16);
    cc2500_set_register(BSCFG    , 0x6C);
    cc2500_set_register(AGCCTRL2 , 0x03);
    cc2500_set_register(AGCCTRL1 , 0x40);  // D4R uses 46 instead of 0x40);
    cc2500_set_register(AGCCTRL0 , 0x91);
    cc2500_set_register(FREND1   , 0x56);
    cc2500_set_register(FREND0   , 0x10);
    cc2500_set_register(FSCAL3   , 0xA9);
    cc2500_set_register(FSCAL2   , 0x05);
    cc2500_set_register(FSCAL1   , 0x00);
    cc2500_set_register(FSCAL0   , 0x11);
    // ???FSTEST   , 0x59);
    cc2500_set_register(TEST2    , 0x88);
    cc2500_set_register(TEST1    , 0x31);
    cc2500_set_register(TEST0    , 0x0B);
    // cc2500_set_register(FIFOTHR  , 0x0F);  // fifo threshold -> tx gdo goes high if 1 byte left
    cc2500_set_register(ADDR     , 0x00);

    // for now just append status
    cc2500_set_register(PKTCTRL1, CC2500_PKTCTRL1_APPEND_STATUS);
    debug("frsky: configure done\n"); debug_flush();
}

void frsky_enter_rxmode(uint8_t channel) {
    cc2500_strobe(RFST_SIDLE);

    cc2500_enter_rxmode();

    // set & do a manual tuning for the given channel
    frsky_tune_channel(channel);

    cc2500_enable_receive();

    // go back to rx mode
    cc2500_strobe(RFST_SRX);
}


void frsky_configure_address(void) {
    // start idle
    cc2500_strobe(RFST_SIDLE);

    // freq offset
    cc2500_set_register(FSCTRL0, storage.frsky_freq_offset);

    // never automatically calibrate, po_timeout count = 64
    // no autotune as(we use our pll map)
    cc2500_set_register(MCSM0, 0x08);

    // set address
    cc2500_set_register(ADDR, storage.frsky_txid[0]);

    // append status, filter by address, autoflush on bad crc, PQT = 0
    cc2500_set_register(PKTCTRL1, CC2500_PKTCTRL1_APPEND_STATUS | CC2500_PKTCTRL1_CRC_AUTOFLUSH | \
                        CC2500_PKTCTRL1_FLAG_ADR_CHECK_01);
}


void frsky_tune_channel(uint8_t ch) {
    // start idle
    cc2500_strobe(RFST_SIDLE);

    // set channel number
    cc2500_set_register(CHANNR, ch);

    // start Self calib:
    cc2500_strobe(RFST_SCAL);

    // wait for scal end
    // either delay_us(800) or check MARCSTATE:
    while (cc2500_get_register(MARCSTATE) != 0x01) {}

    // now FSCAL3..1 shold be set up correctly! yay!
}

void frsky_handle_overflows(void) {
    uint8_t marc_state;

    // fetch marc status
    marc_state = cc2500_get_register(MARCSTATE) & 0x1F;
    if (marc_state == 0x11) {
        debug("frsky: RXOVF\n");
        // flush rx buf
        cc2500_strobe(RFST_SFRX);
        // cc2500_strobe(RFST_SIDLE);
    } else if (marc_state == 0x16) {
        debug("frsky: TXOVF\n");
        // flush tx buf
        cc2500_strobe(RFST_SFTX);
        // cc2500_strobe(RFST_SIDLE);
    }
}

void frsky_calib_pll(void) {
    uint8_t i;
    uint8_t ch;

    debug("frsky: calib pll\n");

    // fine tune offset
    cc2500_set_register(FSCTRL0, storage.frsky_freq_offset);

    debug("frsky: tuning hop[] =");

    // calibrate pll for all channels
    for (i = 0; i < FRSKY_HOPTABLE_SIZE; i++) {
        // reset wdt
        wdt_reset();

        // fetch channel from hop_table:
        ch = storage.frsky_hop_table[i];

        // debug info
        if (i < 9) {
            debug_put_hex8(ch);
            debug_putc(' ');
        }

        // set channel number
        frsky_tune_channel(ch);

        // store pll calibration:
        frsky_calib_fscal1_table[i] = cc2500_get_register(FSCAL1);
    }
    debug_put_newline();

    // only needed once:
    frsky_calib_fscal3 = cc2500_get_register(FSCAL3);
    frsky_calib_fscal2 = cc2500_get_register(FSCAL2);

    // return to idle
    cc2500_strobe(RFST_SIDLE);

    debug("...\nfrsky: calib fscal0 = ");
    debug_put_int8(storage.frsky_freq_offset);
    debug("\nfrsky: calib fscal1:\n");
    for (i = 0; i < 9; i++) {
        debug_put_hex8(frsky_calib_fscal1_table[i]);
        debug_putc(' ');
    }
    debug("...\nfrsky: calib fscal2 = 0x");
    debug_put_hex8(frsky_calib_fscal2);
    debug("\nfrsky: calib fscal3 = 0x");
    debug_put_hex8(frsky_calib_fscal3);
    debug_put_newline();
    debug_flush();

    debug("frsky: calib pll done\n");
}

void frsky_set_channel(uint8_t hop_index) {
    uint8_t ch = storage.frsky_hop_table[hop_index];
    // debug_putc('S'); debug_put_hex8(ch);

    // go to idle
    cc2500_strobe(RFST_SIDLE);

    // fetch and set our stored pll calib data:
    cc2500_set_register(FSCAL3, frsky_calib_fscal3);
    cc2500_set_register(FSCAL2, frsky_calib_fscal2);
    cc2500_set_register(FSCAL1, frsky_calib_fscal1_table[hop_index]);

    // set channel
    cc2500_set_register(CHANNR, ch);
}

void frsky_do_clone_prepare(void) {
    debug("frsky: do clone\n"); debug_flush();

    // set txid to bind channel
    storage.frsky_txid[0] = 0x03;

    // frequency offset to zero(will do auto tune later on)
    storage.frsky_freq_offset = 0;

    frsky_configure();

    // init txid matching
    frsky_configure_address();

    frsky_calib_pll();

    // set up leds:
    led_button_r_off();
    led_button_l_on();
}

void frsky_do_clone_finish(void) {
    // important: stop RF interrupts:
    cc2500_disable_rf_interrupt();

    // save to persistant storage:
    storage_save();

    // done, end up in fancy blink code
    debug("frsky: finished binding. please reset\n");
    led_button_l_on();
}


void frsky_autotune_prepare(void) {
    debug("frsky: autotune\n"); debug_flush();

    // enter RX mode
    frsky_enter_rxmode(0);

    // find best offset:
    storage.frsky_freq_offset = 0;

    debug("frsky: entering bind loop\n"); debug_flush();

    led_button_r_off();

    frsky_state = 0;
    frsky_fscal0_min = 127;
    frsky_fscal0_max = -127;
    frsky_bind_packet_received = 0;
}

uint32_t frsky_autotune_do(void) {
    // search for best fscal 0 match
    // reset wdt
    wdt_reset();

    // handle any ovf conditions
    frsky_handle_overflows();

    // search full range quickly using binary search
    switch (frsky_state) {
        default:
        case (0):
            // init left search:
            storage.frsky_freq_offset = -127;
            frsky_state = 1;
            break;

        case (1):
            // first search quickly through the full range:
            if (storage.frsky_freq_offset < 127-10) {
                storage.frsky_freq_offset += 9;
            } else {
                // done one search, did we receive anything?
                if (frsky_bind_packet_received) {
                    // finished, go to slow search
                    storage.frsky_freq_offset = frsky_fscal0_min - 9;
                    frsky_state = 2;
                } else {
                    // no success, lets try again
                    frsky_state = 0;
                }
            }
            break;

        case (2):
            if (storage.frsky_freq_offset < frsky_fscal0_max+9) {
                storage.frsky_freq_offset++;
            } else {
                // done!
                frsky_state = 5;
            }
            break;
    }

    // go to idle
    cc2500_strobe(RFST_SIDLE);

    // set freq offset
    cc2500_set_register(FSCTRL0, storage.frsky_freq_offset);

    led_button_r_off();

    // go back to RX:
    delay_ms(1);
    cc2500_strobe(RFST_SRX);

    // set timeout
    timeout_set(50);

    led_button_l_on();
    led_button_r_off();

    // debug("tune "); debug_put_int8(storage.frsky_freq_offset);
    // debug_put_newline(); debug_flush();
    uint32_t done = 0;
    while ((!timeout_timed_out()) && (!done)) {
        frsky_packet_received = 0;

        // handle any ovf conditions
        frsky_handle_overflows();

        cc2500_process_packet(&frsky_packet_received, \
                              (volatile uint8_t *)&frsky_packet_buffer, \
                              FRSKY_PACKET_BUFFER_SIZE);

        if (frsky_packet_received) {
            // prepare for next packet:
            frsky_packet_received = 0;
            cc2500_enable_receive();
            cc2500_strobe(RFST_SRX);

            // valid packet?
            if (FRSKY_VALID_PACKET_BIND(frsky_packet_buffer)) {
                // bind packet!
                debug_putc('B');

                // packet received
                frsky_bind_packet_received = 1;

                // this fscal value is done
                done = 1;

                // update min/ max
                frsky_fscal0_min = min(frsky_fscal0_min, storage.frsky_freq_offset);
                frsky_fscal0_max = max(frsky_fscal0_max, storage.frsky_freq_offset);

                // make sure we never read the same packet twice by invalidating packet
                frsky_packet_buffer[0] = 0x00;
            }

            /*debug("[");debug_flush();
    uint8_t cnt;
            for (cnt = 0; cnt < FRSKY_PACKET_BUFFER_SIZE; cnt++) {
                debug_put_hex8(frsky_packet_buffer[cnt]);
                debug_putc(' ');
                debug_flush();
            }
            debug("]\n"); debug_flush();*/
        }
    }
    if (!done) {
        debug_putc('-');
    }
    debug_flush();

    return (frsky_state == 5);
}


void frsky_autotune_finish(void) {
    // set offset to what we found out to be the best:
    int16_t fscal0_calc = (frsky_fscal0_max + frsky_fscal0_min)/ 2;

    debug("\nfrsky: fscal0 ");
    debug_put_int8(frsky_fscal0_min);
    debug(" - ");
    debug_put_int8(frsky_fscal0_max);
    debug_put_newline();
    debug_flush();

    // store new value
    storage.frsky_freq_offset = fscal0_calc;

    cc2500_strobe(RFST_SIDLE);

    // set freq offset
    cc2500_set_register(FSCTRL0, storage.frsky_freq_offset);

    // go back to RX:
    delay_ms(1);
    cc2500_strobe(RFST_SRX);

    debug("frsky: autotune done\n");
    debug("frsky: offset=");
    debug_put_int8(storage.frsky_freq_offset);
    debug_put_newline();
    debug_flush();
}

void frsky_fetch_txid_and_hoptable_prepare(void) {
    debug("frsky: fetching txid + hopt\n"); debug_flush();

    // enter RX mode
    frsky_enter_rxmode(0);

    // clear txid:
    storage.frsky_txid[0] = 0;
    storage.frsky_txid[1] = 0;

    // timeout to wait for packets
    timeout_set(9*3+1);
    frsky_bind_packet_received = 0;
    frsky_bind_packet_hoptable_flags = 0;
}

uint32_t frsky_fetch_txid_and_hoptable_do(void) {
    uint8_t i;

    // fetch hopdata array
    // reset wdt
    wdt_reset();

    // handle any ovf conditions
    frsky_handle_overflows();

    // FIXME: this should be handled in a cleaner way.
    // as this is just for binding, stay with this fix for now...
    if (timeout_timed_out()) {
        // do diversity
        // cc2500_switch_antenna();

        debug_putc('m');

        // next packet should be there ein 9ms
        // if no packet for 3*9ms -> reset rx chain:
        timeout_set(3*9+1);

        // re-prepare for next packet:
        cc2500_strobe(RFST_SIDLE);
        // TESTME: moved to rx_sleep....
        // delay_ms(1);
        frsky_packet_received = 0;
        cc2500_rx_sleep();
        cc2500_enable_receive();
        cc2500_strobe(RFST_SRX);
    }

    // process incoming data
    cc2500_process_packet(&frsky_packet_received, (volatile uint8_t *)&frsky_packet_buffer, \
                          FRSKY_PACKET_BUFFER_SIZE);

    if (frsky_packet_received) {
        debug_putc('p');

        // prepare for next packet:
        frsky_packet_received = 0;
        cc2500_enable_receive();
        cc2500_strobe(RFST_SRX);


#if FRSKY_DEBUG_BIND_DATA
        if (FRSKY_VALID_FRAMELENGTH(frsky_packet_buffer)) {
            debug("frsky: RX ");
            debug_flush();
            for (i = 0; i < FRSKY_PACKET_BUFFER_SIZE; i++) {
                debug_put_hex8(frsky_packet_buffer[i]);
                debug_putc(' ');
            }
            debug_put_newline();
        }
#endif  // FRSKY_DEBUG_BIND_DATA


        // do we know our txid yet?
        if (FRSKY_VALID_PACKET_BIND(frsky_packet_buffer)) {
            // next packet should be ther ein 9ms
            // if no packet for 3*9ms -> reset rx chain:
            timeout_set(3*9+1);

            debug_putc('B');
            if ((storage.frsky_txid[0] == 0) && (storage.frsky_txid[1] == 0)) {
                // no! extract this
                storage.frsky_txid[0] = frsky_packet_buffer[3];
                storage.frsky_txid[1] = frsky_packet_buffer[4];
                // debug
                debug("\nfrsky: got txid 0x");
                debug_put_hex8(storage.frsky_txid[0]);
                debug_put_hex8(storage.frsky_txid[1]);
                debug_put_newline();
            }

            // this is actually for us
            uint8_t index = frsky_packet_buffer[5];

            // valid bind index?
            if (index/ 5 < MAX_BIND_PACKET_COUNT) {
                // copy data to our hop list:
                for (i = 0; i < 5; i++) {
                    if ((index+i) < FRSKY_HOPTABLE_SIZE) {
                        storage.frsky_hop_table[index+i] = frsky_packet_buffer[6+i];
                    }
                }
                // mark as done: set bit flag for index
                frsky_bind_packet_hoptable_flags |= (1 << (index/ 5));
            } else {
                debug("frsky: invalid bind idx");
                debug_put_uint8(index / 5);
                debug_put_newline();
            }

            // make sure we never read the same packet twice by crc flag
            frsky_packet_buffer[FRSKY_PACKET_BUFFER_SIZE-1] = 0x00;
        }
    }
    debug_put_uint8(frsky_bind_packet_hoptable_flags);
    debug_put_newline();

    debug_flush();

    return (frsky_bind_packet_hoptable_flags == HOPDATA_RECEIVE_DONE);
}

void frsky_fetch_txid_and_hoptable_finish(void) {
#if FRSKY_DEBUG_BIND_DATA
    debug("frsky: hop[] = ");
    for (i = 0; i < FRSKY_HOPTABLE_SIZE; i++) {
        debug_put_hex8(storage.frsky_hop_table[i]);
        debug_putc(' ');
        debug_flush();
    }
#endif  // FRSKY_DEBUG_BIND_DATA
    debug_putc('\n');
    debug_flush();

    // idle
    cc2500_strobe(RFST_SIDLE);
}

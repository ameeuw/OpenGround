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

#include <stdbool.h>
#include <stdint.h>

#include "storage.h"
#include "adc.h"
#include "cc2500.h"

#include "iface_cc2500.h"
#include "frsky.h"
#include "FrSkyD_cc2500.h"

#define FRSKY_D_CHAN_OFFSET 2250

uint16_t counter;

static void frsky2way_build_bind_packet(void);
static void frsky2way_data_frame(void);

static void __attribute__((unused)) frsky2way_init(uint8_t bind)
{
    FRSKY_init_cc2500();

    cc2500_set_register(CC2500_09_ADDR, bind ? 0x03 : storage.frsky_txid[0]);

    cc2500_set_register(CC2500_07_PKTCTRL1, 0x05);
    cc2500_strobe(CC2500_SIDLE);    // Go to idle...
    //
    cc2500_set_register(CC2500_0A_CHANNR, 0x00);
    cc2500_set_register(CC2500_23_FSCAL3, 0x89);
    cc2500_strobe(CC2500_SFRX);
    //#######END INIT########
}

static void __attribute__((unused)) frsky2way_build_bind_packet()
{
    //11 03 01 d7 2d 00 00 1e 3c 5b 78 00 00 00 00 00 00 01
    //11 03 01 19 3e 00 02 8e 2f bb 5c 00 00 00 00 00 00 01
    packet[0] = 0x11;
    packet[1] = 0x03;
    packet[2] = 0x01;
    packet[3] = storage.frsky_txid[0];
    packet[4] = storage.frsky_txid[1];
    uint16_t idx = ((state -FRSKY_BIND) % 10) * 5;
    packet[5] = idx;
    packet[6] = storage.frsky_hop_table[idx++];
    packet[7] = storage.frsky_hop_table[idx++];
    packet[8] = storage.frsky_hop_table[idx++];
    packet[9] = storage.frsky_hop_table[idx++];
    packet[10] = storage.frsky_hop_table[idx++];
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    packet[14] = 0x00;
    packet[15] = 0x00;
    packet[16] = 0x00;
    packet[17] = 0x01;
}

static void __attribute__((unused)) frsky2way_data_frame()
{//pachet[4] is telemetry user frame counter(hub)
    //11 d7 2d 22 00 01 c9 c9 ca ca 88 88 ca ca c9 ca 88 88
    //11 57 12 00 00 01 f2 f2 f2 f2 06 06 ca ca ca ca 18 18
    packet[0] = 0x11;             //Length
    packet[1] = storage.frsky_txid[0];
    packet[2] = storage.frsky_txid[1];
    packet[3] = counter;//
    #if defined TELEMETRY
        packet[4] = telemetry_counter;
    #else
        packet[4] = 0x00;
    #endif

    packet[5] = 0x01;
    //
    packet[10] = 0;
    packet[11] = 0;
    packet[16] = 0;
    packet[17] = 0;

    // fetch adc channel data
    adc_process();

    for(uint8_t i = 0; i < 8; i++)
    {
        uint16_t value = adc_get_channel_packetdata(i, FRSKY_D_CHAN_OFFSET);
        if(i < 4)
        {
            packet[6+i] = value & 0xff;
            packet[10+(i>>1)] |= ((value >> 8) & 0x0f) << (4 *(i & 0x01));
        }
        else
        {
            packet[8+i] = value & 0xff;
            packet[16+((i-4)>>1)] |= ((value >> 8) & 0x0f) << (4 * ((i-4) & 0x01));
        }
    }
}

uint16_t initFrSky_2way()
{
    //Frsky_init_hop();
    packet_count=0;
    #if defined TELEMETRY
        init_frskyd_link_telemetry();
    #endif
    state = FRSKY_BIND_DONE;
    return 10000;
}

void bind_d(void) {
    state = FRSKY_BIND;
}

uint16_t ReadFrSky_2way()
{
    if (state < FRSKY_BIND_DONE)
    {
        frsky2way_build_bind_packet();
        cc2500_enter_txmode();
        cc2500_strobe(CC2500_SIDLE);
        cc2500_set_register(CC2500_0A_CHANNR, 0x00);
        cc2500_set_register(CC2500_23_FSCAL3, 0x89);
        cc2500_strobe(CC2500_SFRX);//0x3A
        cc2500_transmit_packet(packet, packet[0]+1);
        /*
        if(IS_BIND_DONE_on)
            state = FRSKY_BIND_DONE;
        else
        */
            state++;
        return 9000;
    }
    if (state == FRSKY_BIND_DONE)
    {
        state = FRSKY_DATA2;
        frsky2way_init(0);
        counter = 0;
        //BIND_DONE;
    }
    else
        if (state == FRSKY_DATA5)
        {
            cc2500_strobe(CC2500_SRX);//0x34 RX enable
            state = FRSKY_DATA1;
            return 9200;
        }
    counter = (counter + 1) % 188;
    if (state == FRSKY_DATA4)
    {   //telemetry receive
        cc2500_enter_rxmode();
        cc2500_strobe(CC2500_SIDLE);
        cc2500_set_register(CC2500_0A_CHANNR, storage.frsky_hop_table[counter % 47]);
        cc2500_set_register(CC2500_23_FSCAL3, 0x89);
        state++;
        return 1300;
    }
    else
    {
        if (state == FRSKY_DATA1)
        {
            uint8_t len = cc2500_get_register(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (len && len<=(0x11+3))// 20bytes
            {
                cc2500_read_fifo(pkt, len);             //received telemetry packets
                #if defined(TELEMETRY)
                    if(pkt[len-1] & 0x80)
                    {//with valid crc
                        packet_count=0;
                        frsky_check_telemetry(pkt,len); //check if valid telemetry packets and buffer them.
                    }
                #endif
            }
            else
            {
                packet_count++;
                // restart sequence on missed packet - might need count or timeout instead of one missed
                if(packet_count>100)
                {//~1sec
                    packet_count=0;
                    #if defined TELEMETRY
                        telemetry_link=0;//no link frames
                        pkt[6]=0;//no user frames.
                    #endif
                }
            }
            cc2500_enter_txmode();
            cc2500_set_power(); // Set tx_power
        }
        cc2500_strobe(CC2500_SIDLE);
        cc2500_set_register(CC2500_0A_CHANNR, storage.frsky_hop_table[counter % 47]);
        if ( prev_option != option )
        {
            cc2500_set_register(CC2500_0C_FSCTRL0,option);  // Frequency offset hack
            prev_option = option ;
        }
        cc2500_set_register(CC2500_23_FSCAL3, 0x89);
        cc2500_strobe(CC2500_SFRX);
        frsky2way_data_frame();
        cc2500_transmit_packet(packet, packet[0]+1);
        state++;
    }
    return state == FRSKY_DATA4 ? 7500 : 9000;
}

/*  **************************
    * By Midelic on RCGroups *
    **************************
    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Multiprotocol is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    gnu general public license for more details.

    you should have received a copy of the gnu general public license
    along with multiprotocol.  if not, see <http://www.gnu.org/licenses/>.
*/

#include "led.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "adc.h"
#include "storage.h"
#include "cc2500.h"
#include "iface_cc2500.h"
#include "frsky.h"
#include "FrSkyX_cc2500.h"

#define FRSKY_X_CHAN_OFFSET 1025

uint8_t chanskip;
// uint8_t seq_last_sent;
// uint8_t seq_last_rcvd;
uint8_t hopping_frequency_no = 0;
uint8_t FrX_send_seq;
uint8_t FrX_receive_seq;

static void frskyX_init(void);
static void frskyX_build_bind_packet(void);
static void frskyX_data_frame(void);
static bool is_eu(void);

static void __attribute__((unused)) set_start(uint8_t ch )
{
    cc2500_strobe(CC2500_SIDLE);
    cc2500_set_register(CC2500_25_FSCAL1, cal_data[ch]);
    cc2500_set_register(CC2500_0A_CHANNR, storage.frsky_hop_table[ch]);
}

static void __attribute__((unused)) frskyX_init()
{
    FRSKY_init_cc2500();
    for(uint8_t c=0;c < FRSKY_HOPTABLE_SIZE;c++)
    {//calibrate hop channels
        cc2500_strobe(CC2500_SIDLE);
        cc2500_set_register(CC2500_0A_CHANNR,storage.frsky_hop_table[c]);
        cc2500_strobe(CC2500_SCAL);
        while (cc2500_get_register(CC2500_35_MARCSTATE) != 0x01) {}
        cal_data[c] = cc2500_get_register(CC2500_25_FSCAL1);
    }
    //#######END INIT########
}

static void __attribute__((unused)) initialize_data(uint8_t adr)
{
    cc2500_set_register(CC2500_0C_FSCTRL0,option);  // Frequency offset hack
    cc2500_set_register(CC2500_18_MCSM0,    0x8);
    cc2500_set_register(CC2500_09_ADDR, adr ? 0x03 : storage.frsky_txid[0]);
    cc2500_set_register(CC2500_07_PKTCTRL1,0x05);
}

//**CRC**
const uint16_t CRC_Short[]={
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7 };
static uint16_t CRCTable(uint8_t val)
{
    uint16_t word ;
    word = CRC_Short[val&0x0F] ;
    val /= 16 ;
    return word ^ (0x1081 * val) ;
}
static uint16_t __attribute__((unused)) crc_x(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for (uint8_t i=0; i < len; i++)
        crc = (crc<<8) ^ CRCTable((uint8_t)(crc>>8) ^ *data++);
    return crc;
}

static bool is_eu()
{
    return (storage.model[storage.current_model].protocol == FRSKY_X_EU);
}

static void __attribute__((unused)) frskyX_build_bind_packet()
{
    uint8_t pkg_len = 0x1D;
    uint8_t limit = 28 ;
    if (is_eu()) {
        pkg_len = 0x20;
        limit = 31;
    }

    packet[0] = pkg_len;
    packet[1] = 0x03;
    packet[2] = 0x01;
    //
    packet[3] = storage.frsky_txid[0];
    packet[4] = storage.frsky_txid[1];
    int idx = ((state -FRSKY_BIND) % 10) * 5;
    packet[5] = idx;
    packet[6] = storage.frsky_hop_table[idx++];
    packet[7] = storage.frsky_hop_table[idx++];
    packet[8] = storage.frsky_hop_table[idx++];
    packet[9] = storage.frsky_hop_table[idx++];
    packet[10] = storage.frsky_hop_table[idx++];
    packet[11] = 0x02;
    packet[12] = storage.model[storage.current_model].rx_number;
    //
    memset(&packet[13], 0, limit - 13);
    uint16_t lcrc = crc_x(&packet[3], limit-3);
    //
    packet[limit++] = lcrc >> 8;
    packet[limit] = lcrc;
    //
}

static void __attribute__((unused)) frskyX_data_frame()
{
    //0x1D 0xB3 0xFD 0x02 0x56 0x07 0x15 0x00 0x00 0x00 0x04 0x40 0x00 0x04 0x40 0x00 0x04 0x40 0x00 0x04 0x40 0x08 0x00 0x00 0x00 0x00 0x00 0x00 0x96 0x12
    //
    static uint8_t lpass;
    uint16_t chan_0 ;
    uint16_t chan_1 ;
    uint8_t startChan = 0;
    //
    packet[0] = is_eu() ? 0x20 : 0x1D ; // LBT or FCC
    packet[1] = storage.frsky_txid[0];
    packet[2] = storage.frsky_txid[1];
    packet[3] = 0x02;
    //
    packet[4] = (chanskip<<6)|hopping_frequency_no;
    packet[5] = chanskip>>2;
    packet[6] = storage.model[storage.current_model].rx_number;
    //packet[7] = FLAGS 00 - standard packet
    //10, 12, 14, 16, 18, 1A, 1C, 1E - failsafe packet
    //20 - range check packet
    packet[7] = 0;
    packet[8] = 0;
    //
    if ( lpass & 1 )
        startChan += 8 ;

    adc_process();

    for(uint8_t i = 0; i <12 ; i+=3)
    {//12 bytes
        chan_0 = adc_get_channel_packetdata(startChan, FRSKY_X_CHAN_OFFSET);
        if(lpass & 1 )
            chan_0+=2048;
        startChan+=1;
        //
        chan_1 = adc_get_channel_packetdata(startChan, FRSKY_X_CHAN_OFFSET);
        if(lpass & 1 )
            chan_1+= 2048;
        startChan+=1;
        //
        packet[9+i] = chan_0 & 0xff;//3 bytes*4
        packet[9+i+1]=(((chan_0>>8) & 0x0F)|(chan_1 << 4));
        packet[9+i+2]=chan_1>>4;
    }

    packet[21] = (FrX_receive_seq << 4) | FrX_send_seq ;//8 at start

    if( true )// in X8 mode send only 8ch every 9ms
        lpass = 0 ;
    else
        lpass += 1 ;

    uint8_t limit = is_eu() ? 31 : 28 ;
    for (uint8_t i=22;i<limit;i++)
        packet[i]=0;
    uint16_t lcrc = crc_x(&packet[3], limit-3);

    packet[limit++]=lcrc>>8;//high byte
    packet[limit]=lcrc;//low byte
}

uint16_t ReadFrSkyX()
{
    uint32_t len;

    switch(state)
    {
        default:
            if (!(state % 100))
                led_button_r_off();
            cc2500_enter_txmode();
            set_start(47);
            cc2500_set_power();
            cc2500_strobe(CC2500_SFRX);
            frskyX_build_bind_packet();
            cc2500_strobe(CC2500_SIDLE);
            cc2500_transmit_packet(packet, packet[0]+1);
            /*
            state = FRSKY_BIND_DONE;
            if(IS_BIND_DONE_on)
                state = FRSKY_BIND_DONE;
            else
            */
                state++;
                if (!(state % 500))
                    led_button_r_on();
                if (state == FRSKY_BIND_DONE)
                    state = FRSKY_BIND;
            return 9000;
        case FRSKY_BIND_DONE:
            initialize_data(0);
            hopping_frequency_no=0;
            //BIND_DONE;
            state++;
            break;
        case FRSKY_DATA1:
            if ( prev_option != option )
            {
                cc2500_set_register(CC2500_0C_FSCTRL0,option);  // Frequency offset hack
                prev_option = option ;
            }
            cc2500_enter_txmode();
            set_start(hopping_frequency_no);
            cc2500_set_power();
            cc2500_strobe(CC2500_SFRX);
            hopping_frequency_no = (hopping_frequency_no+chanskip)%47;
            cc2500_strobe(CC2500_SIDLE);
            cc2500_transmit_packet(packet, packet[0]+1);
            //
//          frskyX_data_frame();
            state++;
            return 5200;
        case FRSKY_DATA2:
            cc2500_enter_rxmode();
            cc2500_strobe(CC2500_SIDLE);
            state++;
            return 200;
        case FRSKY_DATA3:
            cc2500_strobe(CC2500_SRX);
            state++;
            return 3100;
        case FRSKY_DATA4:
            len = cc2500_get_register(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (len && (len<=(0x0E + 3)))               //Telemetry frame is 17
            {
                packet_count=0;
                cc2500_read_fifo(pkt, len);
                #if defined TELEMETRY
                    frsky_check_telemetry(pkt,len); //check if valid telemetry packets
                    //parse telemetry packets here
                    //The same telemetry function used by FrSky(D8).
                #endif
            }
            else
            {
                packet_count++;
                // restart sequence on missed packet - might need count or timeout instead of one missed
                if(packet_count>100)
                {//~1sec
//                  seq_last_sent = 0;
//                  seq_last_rcvd = 8;
                    FrX_send_seq = 0x08 ;
//                  FrX_receive_seq = 0 ;
                    packet_count=0;
                    #if defined TELEMETRY
                        telemetry_lost=1;
                    #endif
                }
                cc2500_strobe(CC2500_SFRX);         //flush the RXFIFO
            }
            frskyX_data_frame();
            if ( FrX_send_seq != 0x08 )
            {
                FrX_send_seq = ( FrX_send_seq + 1 ) & 0x03 ;
            }
            state = FRSKY_DATA1;
            return 500;
    }
    return 1;
}

uint16_t initFrSkyX()
{
    //Frsky_init_hop();
    packet_count=0;
    /* FIXME: random
    while(!chanskip)
        chanskip=random(0xfefefefe)%47;
        */
    chanskip=4;

    //for test***************
    //rx_tx_addr[3]=0xB3;
    //rx_tx_addr[2]=0xFD;
    //************************
    frskyX_init();
    state = FRSKY_DATA1;
    initialize_data(0);

//  seq_last_sent = 0;
//  seq_last_rcvd = 8;
    FrX_send_seq = 0x08 ;
    FrX_receive_seq = 0 ;
    return 10000;
}

void bind_x(void) {
    state = FRSKY_BIND;
}

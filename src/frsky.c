/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/ or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/ licenses/>.

    author: fishpepper <AT> gmail.com
*/

#include "main.h"
#include <string.h>
#include <stdio.h>
#include "frsky.h"
#include "debug.h"
#include "timeout.h"
#include "led.h"
#include "delay.h"
#include "wdt.h"
#include "io.h"
#include "clocksource.h"
#include "storage.h"
#include "adc.h"
#include "telemetry.h"

#include <libopencm3/stm32/timer.h>

#include "storage.h"
#include "FrSkyD_cc2500.h"
#include "FrSkyX_cc2500.h"


uint16_t (*irq_handler)(void);
/*
void Frsky_init_hop(void)
{
    uint8_t val;
    uint8_t channel = 0; // 0-7 //rx_tx_addr[0]&0x07;
    uint8_t channel_spacing = 128; // 64-192 //rx_tx_addr[1];
    //Filter bad tables
    if(channel_spacing<0x02) channel_spacing+=0x02;
    if(channel_spacing>0xE9) channel_spacing-=0xE7;
    if(channel_spacing%0x2F==0) channel_spacing++;

    hopping_frequency[0]=channel;
    for(uint8_t i=1;i<50;i++)
    {
        channel=(channel+channel_spacing) % 0xEB;
        val=channel;
        if((val==0x00) || (val==0x5A) || (val==0xDC))
            val++;
        hopping_frequency[i]=i>46?0:val;
    }
}
*/

uint8_t packet[40];
uint16_t state;
uint8_t packet_count;
uint8_t option, prev_option;
uint8_t cal_data[FRSKY_HOPTABLE_SIZE];
uint8_t pkt[MAX_PKT];

const uint8_t FRSKY_common_startreg_cc2500_conf[]= {
    CC2500_02_IOCFG0 ,
    CC2500_00_IOCFG2 ,
    CC2500_17_MCSM1 ,
    CC2500_18_MCSM0 ,
    CC2500_06_PKTLEN ,
    CC2500_07_PKTCTRL1 ,
    CC2500_08_PKTCTRL0 ,
    CC2500_3E_PATABLE ,
    CC2500_0B_FSCTRL1 ,
    CC2500_0C_FSCTRL0 , // replaced by option value
    CC2500_0D_FREQ2 ,
    CC2500_0E_FREQ1 ,
    CC2500_0F_FREQ0 ,
    CC2500_10_MDMCFG4 ,
    CC2500_11_MDMCFG3 ,
    CC2500_12_MDMCFG2 ,
    CC2500_13_MDMCFG1 ,
    CC2500_14_MDMCFG0 ,
    CC2500_15_DEVIATN  };

#if defined(FRSKYV_CC2500_INO)
const uint8_t FRSKYV_cc2500_conf[]= {
    /*02_IOCFG0*/    0x06 ,
    /*00_IOCFG2*/    0x06 ,
    /*17_MCSM1*/     0x0c ,
    /*18_MCSM0*/     0x18 ,
    /*06_PKTLEN*/    0xff ,
    /*07_PKTCTRL1*/  0x04 ,
    /*08_PKTCTRL0*/  0x05 ,
    /*3E_PATABLE*/   0xfe ,
    /*0B_FSCTRL1*/   0x08 ,
    /*0C_FSCTRL0*/   0x00 ,
    /*0D_FREQ2*/     0x5c ,
    /*0E_FREQ1*/     0x58 ,
    /*0F_FREQ0*/     0x9d ,
    /*10_MDMCFG4*/   0xAA ,
    /*11_MDMCFG3*/   0x10 ,
    /*12_MDMCFG2*/   0x93 ,
    /*13_MDMCFG1*/   0x23 ,
    /*14_MDMCFG0*/   0x7a ,
    /*15_DEVIATN*/   0x41  };
#endif

const uint8_t FRSKYD_cc2500_conf[]= {
    /*02_IOCFG0*/    0x06 ,
    /*00_IOCFG2*/    0x06 ,
    /*17_MCSM1*/     0x0c ,
    /*18_MCSM0*/     0x18 ,
    /*06_PKTLEN*/    0x19 ,
    /*07_PKTCTRL1*/  0x04 ,
    /*08_PKTCTRL0*/  0x05 ,
    /*3E_PATABLE*/   0xff ,
    /*0B_FSCTRL1*/   0x08 ,
    /*0C_FSCTRL0*/   0x00 ,
    /*0D_FREQ2*/     0x5c ,
    /*0E_FREQ1*/     0x76 ,
    /*0F_FREQ0*/     0x27 ,
    /*10_MDMCFG4*/   0xAA ,
    /*11_MDMCFG3*/   0x39 ,
    /*12_MDMCFG2*/   0x11 ,
    /*13_MDMCFG1*/   0x23 ,
    /*14_MDMCFG0*/   0x7a ,
    /*15_DEVIATN*/   0x42  };

const uint8_t FRSKYX_cc2500_conf[]= {
    //FRSKYX
    /*02_IOCFG0*/    0x06 ,
    /*00_IOCFG2*/    0x06 ,
    /*17_MCSM1*/     0x0c ,
    /*18_MCSM0*/     0x18 ,
    /*06_PKTLEN*/    0x1E ,
    /*07_PKTCTRL1*/  0x04 ,
    /*08_PKTCTRL0*/  0x01 ,
    /*3E_PATABLE*/   0xff ,
    /*0B_FSCTRL1*/   0x0A ,
    /*0C_FSCTRL0*/   0x00 ,
    /*0D_FREQ2*/     0x5c ,
    /*0E_FREQ1*/     0x76 ,
    /*0F_FREQ0*/     0x27 ,
    /*10_MDMCFG4*/   0x7B ,
    /*11_MDMCFG3*/   0x61 ,
    /*12_MDMCFG2*/   0x13 ,
    /*13_MDMCFG1*/   0x23 ,
    /*14_MDMCFG0*/   0x7a ,
    /*15_DEVIATN*/   0x51  };
const uint8_t FRSKYXEU_cc2500_conf[]= {
    /*02_IOCFG0*/    0x06 ,
    /*00_IOCFG2*/    0x06 ,
    /*17_MCSM1*/     0x0E ,
    /*18_MCSM0*/     0x18 ,
    /*06_PKTLEN*/    0x23 ,
    /*07_PKTCTRL1*/  0x04 ,
    /*08_PKTCTRL0*/  0x01 ,
    /*3E_PATABLE*/   0xff ,
    /*0B_FSCTRL1*/   0x08 ,
    /*0C_FSCTRL0*/   0x00 ,
    /*0D_FREQ2*/     0x5c ,
    /*0E_FREQ1*/     0x80 ,
    /*0F_FREQ0*/     0x00 ,
    /*10_MDMCFG4*/   0x7B ,
    /*11_MDMCFG3*/   0xF8 ,
    /*12_MDMCFG2*/   0x03 ,
    /*13_MDMCFG1*/   0x23 ,
    /*14_MDMCFG0*/   0x7a ,
    /*15_DEVIATN*/   0x53  };

const uint8_t FRSKY_common_end_cc2500_conf[][2]= {
    { CC2500_19_FOCCFG,   0x16 },
    { CC2500_1A_BSCFG,    0x6c },
    { CC2500_1B_AGCCTRL2, 0x43 },
    { CC2500_1C_AGCCTRL1, 0x40 },
    { CC2500_1D_AGCCTRL0, 0x91 },
    { CC2500_21_FREND1,   0x56 },
    { CC2500_22_FREND0,   0x10 },
    { CC2500_23_FSCAL3,   0xa9 },
    { CC2500_24_FSCAL2,   0x0A },
    { CC2500_25_FSCAL1,   0x00 },
    { CC2500_26_FSCAL0,   0x11 },
    { CC2500_29_FSTEST,   0x59 },
    { CC2500_2C_TEST2,    0x88 },
    { CC2500_2D_TEST1,    0x31 },
    { CC2500_2E_TEST0,    0x0B },
    { CC2500_03_FIFOTHR,  0x07 },
    { CC2500_09_ADDR,     0x00 } };

void FRSKY_init_cc2500()
{
    const uint8_t *ptr;

    debug("FRSKY_init_cc2500 proto ");
    debug_put_uint8(storage.model[storage.current_model].protocol);
    debug("\n");
    debug_flush();

    switch (storage.model[storage.current_model].protocol) {
        default:
        case FRSKY_X:
            ptr = FRSKYX_cc2500_conf;
            break;
        case FRSKY_X_EU:
            ptr = FRSKYXEU_cc2500_conf;
            break;
        case FRSKY_D:
            ptr = FRSKYD_cc2500_conf;
            break;
    };

    option = storage.frsky_freq_offset;

    for(uint8_t i=0;i<19;i++)
    {
        uint8_t reg=FRSKY_common_startreg_cc2500_conf[i];
        uint8_t val=ptr[i];
        if(reg==CC2500_0C_FSCTRL0)
            val=option;
        cc2500_set_register(reg,val);
    }
    prev_option = option ;      // Save option to monitor FSCTRL0 change
    for(uint8_t i=0;i<17;i++)
    {
        uint8_t reg=FRSKY_common_end_cc2500_conf[i][0];
        uint8_t val=FRSKY_common_end_cc2500_conf[i][1];
        cc2500_set_register(reg,val);
    }
    cc2500_enter_txmode();
    cc2500_set_power();
    cc2500_strobe(CC2500_SIDLE);    // Go to idle...
}

void frsky_init(void) {
    timer_disable_irq(TIM3, TIM_DIER_UIE);

    //telemetry_init();
    cc2500_init();

    // check if spi is working properly
    /*if (!frsky_check_transceiver()) {
        // no cc2500 detected - abort
        debug("frsky: no cc2500 detected. abort\n");
        debug_flush();
        return;
    }
    */

    if (storage.model[storage.current_model].protocol == FRSKY_X) {
        initFrSkyX();
        irq_handler = ReadFrSkyX;
    } else if (storage.model[storage.current_model].protocol == FRSKY_D) {
        initFrSky_2way();
        irq_handler = ReadFrSky_2way;
    }

    frsky_init_timer();
    timer_set_period(TIM3, 9000-1);
    //timer_enable_irq(tim3, tim_dier_uie);
    frsky_tx_set_enabled(1);
}

void TIM3_IRQHandler(void) {
    if (timer_get_flag(TIM3, TIM_SR_UIF)) {
        // clear flag (note: this should never be done at the end of the isr)
        timer_clear_flag(TIM3, TIM_SR_UIF);
        if (irq_handler)
            timer_set_period(TIM3, irq_handler()-1);
    }
}

void frsky_enter_bindmode(void) {
    debug("frsky: bind ===============\n"); debug_flush();
    frsky_tx_set_enabled(0);
    if (storage.model[storage.current_model].protocol == FRSKY_X) {
        debug("frsky: bind x\n"); debug_flush();
        bind_x();
    } else if (storage.model[storage.current_model].protocol == FRSKY_D) {
        bind_d();
        debug("frsky: bind d\n"); debug_flush();
    }
    frsky_tx_set_enabled(1);
}

void frsky_init_timer(void) {
    // tim3 clock enable
    rcc_periph_clock_enable(RCC_TIM3);

    // init timer3 for 9ms
    timer_reset(TIM3);

    // enable the tim3 gloabal interrupt
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, NVIC_PRIO_FRSKY);

    // compute prescaler value
    // we want one isr every 9ms
    // setting tim_period to 9000 will reuqire
    // a prescaler so that one timer tick is 1us (1mhz)
    uint16_t prescaler = (uint16_t) (rcc_timer_frequency  / 1000000) - 1;

    // time base as calculated above
    timer_set_prescaler(TIM3, prescaler);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // timer should count with 1mhz thus 9000 ticks = 9ms
    timer_set_period(TIM3, 9000-1);

    // do not enable int yet!

    // enable timer
    timer_enable_counter(TIM3);
}

void frsky_tx_set_enabled(uint32_t enabled) {
    // tim interrupts enable? -> tx active
    if (enabled) {
        // enable isr
        timer_enable_irq(TIM3, TIM_DIER_UIE);
    } else {
        // stop isr
        timer_disable_irq(TIM3, TIM_DIER_UIE);
        // make sure last packet was sent
        delay_ms(20);
    }
}

uint8_t frsky_check_transceiver(void) {
    debug("frsky: partinfo\n"); debug_flush();

    uint8_t partnum = 0, version = 0;
    // start idle
    cc2500_strobe(RFST_SIDLE);

    // check version:
    debug("frsky: cc2500 partnum 0x");
    partnum = cc2500_get_register_burst(partnum);
    debug_put_hex8(partnum);

    debug(" version 0x");
    version = cc2500_get_register_burst(version);
    debug_put_hex8(version);
    debug_put_newline();

    if (cc2500_partnum_valid(partnum, version)) {
        debug("frsky: got valid part and version info\n");
        debug_flush();
        return 1;
    }

    // else
    debug("frsky: got invalid part and version info?!\n");
    debug_flush();
    return 0;
}

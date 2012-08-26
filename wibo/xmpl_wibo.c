/* Copyright (c) 2011 Axel Wachtler, Daniel Thiele
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

/* $Id$ */
/**
 * @file
 * @brief Wireless bootloader example application
 * It blinks a LED so that you can see the success of your
 * wireless bootloading process. Additionally it receives the
 * command to jump back into bootloader.
 *
 * @author Daniel Thiele
 *         Dietzsch und Thiele PartG
 *         Bautzner Str. 67 01099 Dresden
 *         www.ib-dt.de daniel.thiele@ib-dt.de
 *
 * @ingroup grpAppWiBo
 */

 /* avr-libc inclusions */
#include <avr/io.h>
#include <util/delay.h>

/* uracoli inclusions */
#include <board.h>
#include <ioutil.h>
#include <radio.h>

/* project inclusions */
#include "wibocmd.h"


#ifndef BLINKPERIOD
#define BLINKPERIOD (125)
#endif

#ifndef BOOTOFFSET
#define BOOTOFFSET (0x0000UL)
#endif

/*
 * Function to jump into bootloader section
 *
 * BOOTOFFSET is the byte-address in flashmem
 */
const void (*bootl)(void) = (void*)BOOTOFFSET;

/* IEEE802.15.4 parameters for communication */
static node_config_t nodeconfig;

void usr_radio_error(radio_error_t err)
{
}

void usr_radio_irq(uint8_t cause)
{
}

uint8_t * usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi,
		int8_t ed, uint8_t crc_fail)
{
	switch( ((wibocmd_hdr_t*)frm)->command){

	case WIBOCMDCODE_XMPLJBOOTL:
		//bootl();				/* jump to */
        LED_SET(0);
		jump_to_bootloader();
        LED_CLR(0);
		break;

	case WIBOCMDCODE_XMPLLED:
		if(((wibocmd_xmplled_t*)frm)->state){
			LED_SET(((wibocmd_xmplled_t*)frm)->led);
		}else{
			LED_CLR(((wibocmd_xmplled_t*)frm)->led);
		}
		break;

	default :
		/* unknown or unhandled command */
		break;
	};
	return frm;
}

void usr_radio_tx_done(radio_tx_done_t status)
{
	if (TX_OK == status) {
	} else {
		/* TODO handle error */
	}
}

int main()
{
	volatile uint32_t cnt=0;
	static uint8_t rxbuf[128];

#if defined(NODECONFIG_STATIC)
	nodeconfig.channel = 11;
	nodeconfig.pan_id = 0x01;
	nodeconfig.short_addr = 0x03;
#else
	get_node_config(&nodeconfig);
#endif

	LED_INIT();

	radio_init(rxbuf, 128);

	radio_set_param(RP_CHANNEL(nodeconfig.channel));
	radio_set_param(RP_PANID(nodeconfig.pan_id));
	radio_set_param(RP_SHORTADDR(nodeconfig.short_addr));
	radio_set_param(RP_IDLESTATE(STATE_RXAUTO));

	sei();

	for(;;)
    {
        LED_SET(0);
        _delay_ms(BLINKPERIOD);
        LED_CLR(0);
        _delay_ms(BLINKPERIOD);
	}
}

/* EOF */

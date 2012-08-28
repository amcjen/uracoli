/* Copyright (c) 2010, 2011, 2012 Daniel Thiele, Axel Wachtler
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
 * @brief Wireless bootloader IEEE802.15.4 communication module
 * Based on radio-layer of uracoli, manage commands located above
 * MAC-Layer
 *
 * For data integrity, a checksum is tracked inside the host as
 * well as on each node taking part in flashing process for every
 * slice of data fed. It can be checked for each node of
 * the network separately.
 *
 *   Polynomial: x^16 + x^12 + x^5 + 1 (0x8408)
 *   This is the CRC used by PPP and IrDA.
 *
 *
 * @author Daniel Thiele
 *         Dietzsch und Thiele PartG
 *         Bautzner Str. 67 01099 Dresden
 *         www.ib-dt.de daniel.thiele@ib-dt.de
 *
 * @ingroup grpAppWiBo
 */

/* avr-libc inclusions */
#include <util/crc16.h>

/* uracoli inclusions */
#include <board.h>
#include <transceiver.h>
#include <radio.h>

/* project inclusions */
#include "wibocmd.h"
#include "wibohost.h"

/* collect data to send here */
static uint8_t txbuf[MAX_FRAME_SIZE];

/* running checksum to calculate over all send data packets (via feed())
 * to be set to zero on start of data stream (reset())
 */
static uint16_t datacrc = 0x0000;

/* frame receive buffer */
static uint8_t rxbuf[MAX_FRAME_SIZE];

static node_config_t nodeconfig;

/******************* uracoli callback functions ***************/

/*
 * \brief Deliver node configuration, IEEE802.15.4 parameters
 *
 * @return Pointer to node configuration structure as defined in uracoli
 */
node_config_t* wibohost_getnodecfg(void)
{
	return &nodeconfig;
}

/*
 * \brief Callback of uracoli radio-layer
 */
void usr_radio_error(radio_error_t err)
{
	/* empty */
}

/*
 * \brief Callback of uracoli radio-layer
 */
void usr_radio_irq(uint8_t cause)
{
	/* empty */
}

/*
 * \brief Callback of uracoli radio-layer
 */
uint8_t * usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi,
		int8_t ed, uint8_t crc_fail)
{
	wibocmd_pingreply_t *pr = (wibocmd_pingreply_t*) frm;

	/* decode command code */
	if (WIBOCMDCODE_PINGREPLY == pr->hdr.command) {
		cb_wibohost_pingreply(pr, lqi, ed);
	} else {
		/* unknown command, skip the frame */
	}

	return frm;
}

/*
 * \brief Callback of uracoli radio-layer
 */
void usr_radio_tx_done(radio_tx_done_t status)
{
	if (TX_OK == status) {
	} else {
		/* TODO handle error */
	}
}

/*
 * \brief Initialize communication layer
 * - Read node configuration from NVRAM
 * - Initialize uracoli radio-layer
 */
void wibohost_init(void)
{
#if defined(NODECONFIG_STATIC)
	/* for development purposes */
	nodeconfig.channel = 11;
	nodeconfig.pan_id = 0x01;
	nodeconfig.short_addr = 0x05;
#else
	get_node_config(&nodeconfig);
#endif
	radio_init(rxbuf, MAX_FRAME_SIZE);
	radio_set_param(RP_PANID(nodeconfig.pan_id));
	radio_set_param(RP_SHORTADDR(nodeconfig.short_addr));
	radio_set_param(RP_IDLESTATE(STATE_RXAUTO));
	
  radio_set_param(RP_CCAMODE(3));
  radio_set_param(RP_TXPWR(15));
}

/*
 * \brief Send a command over the air
 *
 * Construct the header that is the same for all commands and
 * pack into IEEE802.15.4 MPDU
 *
 * @param short_addr IEEE short address
 * @param cmdcode Command code as defined in wibocmd.h
 * @param *data Pointer to buffer of Tx frame (header is mapped onto it here)
 * @param lendata Overall length of Tx frame
 */
static void wibohost_sendcommand(uint16_t short_addr, wibocmdcode_t cmdcode,
		uint8_t *data, uint8_t lendata)
{
	wibocmd_hdr_t *hdr = (wibocmd_hdr_t*) data;

	hdr->FCF = 0x8801; /* short addressing, frame type: data, no ACK requested */
	hdr->command = cmdcode;
	hdr->destpanid = nodeconfig.pan_id;
	hdr->destaddr = short_addr;
	hdr->srcaddr = nodeconfig.short_addr;
	radio_set_state(STATE_TXAUTO);
	radio_send_frame(lendata + 2, data, 1); /* +2: add CRC bytes (FCF) */
}

/*
 * \brief Feed data to a node
 * This data is part of the flash image that shall be programmed to
 * the target device. It can be sliced to any size from outside. The
 * target device collects it and programs its pages as soon as one page
 * size of data is reached.
 *
 * @param short_addr Address of node to feed (or broadcast 0xFFFF)
 * @param *data Pointer to buffer where data is stored
 * @param lendata Length of buffer
 */
void wibohost_feed(uint16_t short_addr, uint8_t *data, uint8_t lendata)
{
	wibocmd_data_t *dat = (wibocmd_data_t*) txbuf;
	uint8_t i;

	for (i = 0; i < lendata; i++) {
		datacrc = _crc_ccitt_update(datacrc, data[i]);
		dat->data[i] = data[i]; /* no use for memcpy() */
	}
	dat->dsize = lendata;
	dat->targmem = 'F';

	wibohost_sendcommand(short_addr, WIBOCMDCODE_DATA, (uint8_t*) dat,
			sizeof(wibocmd_data_t) + lendata);
}

/*
 * \brief Issue command to ping a node
 *
 * @param short_addr The node addressed (broadcast not useful)
 */
void wibohost_ping(uint16_t short_addr)
{
	wibocmd_ping_t *ping = (wibocmd_ping_t*) txbuf;

	wibohost_sendcommand(short_addr, WIBOCMDCODE_PING, (uint8_t*) ping,
			sizeof(wibocmd_ping_t));
}

/*
 * \brief Issue command to force write pagebuffer that is filled partially
 *
 * @param short_addr The node addressed (or broadcast 0xFFFF)
 */
void wibohost_finish(uint16_t short_addr)
{
	wibocmd_finish_t *dat = (wibocmd_finish_t*) txbuf;

	/* no data fields to fill */

	wibohost_sendcommand(short_addr, WIBOCMDCODE_FINISH, (uint8_t*) dat,
			sizeof(wibocmd_finish_t));
}

/*
 * \brief Reset the running checksum for image data
 * Reset our own checksum and all nodes in the network, therefore
 * send a broadcast. Otherwise a separate CRC for each node
 * would have to be hold here
 *
 */
void wibohost_reset(void)
{
	wibocmd_reset_t *dat = (wibocmd_reset_t*) txbuf;

	/* no data fields to fill */

	wibohost_sendcommand(0xFFFF, WIBOCMDCODE_RESET, (uint8_t*) dat,
			sizeof(wibocmd_reset_t));

	/* reset checksum */
	datacrc = 0x0000;
}

/*
 * \brief Deliver the checksum of the data fed since last wibohost_reset()
 *
 * @return CRC-16 checksum
 */
uint16_t wibohost_getcrc(void)
{
	return datacrc;
}

/*
 * \brief Issue command to exit bootloader and jump into
 * application (Vector 0x0000)
 *
 * @param short_addr The node addressed (or broadcast 0xFFFF)
 */
void wibohost_exit(uint16_t short_addr)
{
	wibocmd_exit_t *dat = (wibocmd_exit_t*) txbuf;

	/* no data fields to fill */

	wibohost_sendcommand(short_addr, WIBOCMDCODE_EXIT, (uint8_t*) dat,
			sizeof(wibocmd_exit_t));
}

/*
 * \brief Issue command to jump into bootloader
 * This is used for example application that demonstrates the
 * ability to jump between application and bootloader by air
 * commands entirely.
 * Usually the application itself is responsible for jumping
 * into bootloader, not the bootloader host application. The
 * command here is for demonstration purposes only.
 *
 * @param short_addr The node addressed (or broadcast 0xFFFF)
 */
void wibohost_xmpljbootl(uint16_t short_addr)
{
	wibocmd_xmpljbootl_t *dat = (wibocmd_xmpljbootl_t*) txbuf;

	wibohost_sendcommand(short_addr, WIBOCMDCODE_XMPLJBOOTL, (uint8_t*) dat,
			sizeof(wibocmd_xmpljbootl_t));
}

/* EOF */

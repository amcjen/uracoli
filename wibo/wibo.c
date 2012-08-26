/* Copyright (c) 2010 Axel Wachtler
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
 * @brief Wireless bootloader application, resides in bootloader section
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
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <string.h>
#if defined(NODECONFIG_SPECIAL_IBDT)
#include <avr/eeprom.h>
#endif

/* uracoli inclusions */
#include <board.h>
#include <ioutil.h>
#include <transceiver.h>

/* project inclusions */
#include "wibocmd.h"

#define _SW_VERSION_ (0x01)

/* serial debug printout via USART0, using uracoli ioutil/hif module */
#ifndef SERIALDEBUG
#define SERIALDEBUG (0)
#endif

#if SERIALDEBUG > 0
#include <avr/interrupt.h>
#include <hif.h>
#define BAUDRATE (38400UL)
#endif

/* incoming frames are collected here */
static uint8_t rxbuf[MAX_FRAME_SIZE];

#define PAGEBUFSIZE (SPM_PAGESIZE)

/* the only outgoing command, create in SRAM here
 * the values assigned here never have to changed
 */
static wibocmd_pingreply_t pingrep = {
		.hdr.command = WIBOCMDCODE_PINGREPLY,
		.hdr.FCF = 0x8841, 				/* short addressing, frame type: data, no ACK requested */
		.pagesize = PAGEBUFSIZE,
		.swversion = _SW_VERSION_,
		.boardname = BOARD_NAME,
};

/* collect memory page data here */
static uint8_t pagebuf[PAGEBUFSIZE];

/* IEEE802.15.4 parameters for communication */
static node_config_t nodeconfig;


/*
 * \brief Program a page
 * (1) Erase the page containing at the given address
 * (2) Fill the page buffer
 * (3) Write the page buffer
 * (4) Wait for finish
 *
 * @param addr The address containing the page to program
 * @param *buf Pointer to buffer of page content
 */
static inline void boot_program_page(uint32_t addr, uint8_t *buf)
{
	uint16_t i;

#if SERIALDEBUG > 0
	printf("Write Flash addr=%04lX", addr);
	for (i = 0; i < SPM_PAGESIZE; i++) {
		if (!(i % 16)) {
			putchar('\n');
			putchar(' ');
			putchar(' ');
		}
		printf(" %02X", buf[i]);
	}
	putchar('\n');
#endif

	boot_page_erase(addr);
	boot_spm_busy_wait(); /* wait the previous page write */

	i = SPM_PAGESIZE;
	do {
		/* Set up little-endian word */
		uint16_t w = *buf++;
		w += (*buf++) << 8;

		boot_page_fill (addr + SPM_PAGESIZE - i, w);

		i -= 2;
	}while(i);

	boot_page_write(addr);
	boot_spm_busy_wait(); /* wait the previous erase */
}

#if SERIALDEBUG > 0

/*
 * \brief Make putc function avr-libc compatible to make use of avr-libc printf/scanf facility
 */
static inline int hif_putc_libcwrapper(char c, FILE *stream)
{
	hif_putc((uint8_t)c);
	return EOF;
}

/*
 * \brief Make getc function avr-libc compatible to make use of avr-libc printf/scanf facility
 */
static inline int hif_getc_libcwrapper(FILE *stream)
{
	uint16_t c = hif_getc();
	return (c&0xFF00)?EOF:(c&0x00FF);
}

static FILE usart0_stdio = FDEV_SETUP_STREAM(hif_putc_libcwrapper, hif_getc_libcwrapper, _FDEV_SETUP_RW);
#endif

int main(void) __attribute__((noreturn));

int main()
{
	void (*app)(void) = 0x0000;

	/* do not initialize variables to save code space, BSS segment sets all variables to zero
	 *
	 * compiler throws warning, please ignore
	 */
	uint8_t *ptr;
	uint8_t i;
	uint16_t datacrc; 			/* checksum for received data */
	uint16_t addr;
	uint16_t pagebufidx;

	LED_INIT();
	LED_SET(1);

#if defined(NODECONFIG_STATIC)
	nodeconfig.channel = 11;
	nodeconfig.pan_id = 0x01;
	nodeconfig.short_addr = 0x03;
#elif defined(NODECONFIG_SPECIAL_IBDT)
	nodeconfig.pan_id = eeprom_read_word((uint16_t*)0x0000);
	nodeconfig.short_addr = eeprom_read_word((uint16_t*)0x0002);
	nodeconfig.channel = eeprom_read_word((uint16_t*)0x0004);
#else
	get_node_config(&nodeconfig);
#endif

	trx_io_init(DEFAULT_SPI_RATE);
	TRX_RESET_LOW();
	TRX_SLPTR_LOW();
	TRX_RESET_HIGH();

	trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF);


	/* BAD: better way would be to abstract for all boards even if it makes no sense (megaRF variants) */
#if defined(DI_TRX_IRQ)
	DI_TRX_IRQ();
#endif

#if (RADIO_TYPE == RADIO_AT86RF230A) || (RADIO_TYPE == RADIO_AT86RF230B)
	trx_reg_write(RG_PHY_TX_PWR, 0x80); /* set TX_AUTO_CRC bit, and TX_PWR = max */
#else
	trx_reg_write(RG_TRX_CTRL_1, 0x20); /* set TX_AUTO_CRC bit */
#endif

	/* setup network addresses for auto modes */
	pingrep.hdr.destpanid = nodeconfig.pan_id;
	pingrep.hdr.srcaddr = nodeconfig.short_addr;

	trx_set_panid(nodeconfig.pan_id);
	trx_set_shortaddr(nodeconfig.short_addr);

	trx_reg_read(RG_IRQ_STATUS);
	
#if RADIO_TYPE == RADIO_ATMEGA128RFA1_A ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_B ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_C ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_D
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_RX_END | TRX_IRQ_TX_END);
#else
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_TRX_END);
#endif
	trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);

#if SERIALDEBUG > 0

	hif_init(BAUDRATE);

	stdin = stdout = stderr = &usart0_stdio;
	sei();
	puts("WIBO Bootlapp Serial Debug");
	printf("PANID: %04X SHORTADDR: %04X\n", nodeconfig.pan_id, nodeconfig.short_addr);
#endif

	for (;;) {
		LED_CLR(1);

#if RADIO_TYPE == RADIO_ATMEGA128RFA1_A ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_B ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_C ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_D
		while (!(trx_reg_read(RG_IRQ_STATUS) & TRX_IRQ_RX_END));
		trx_reg_write(RG_IRQ_STATUS, TRX_IRQ_RX_END); /* clear the flag */
#else
		while (!(trx_reg_read(RG_IRQ_STATUS) & TRX_IRQ_TRX_END));
#endif
		trx_frame_read(rxbuf, 128, &i); /* dont use LQI, write into tmp variable */

		LED_SET(1); /* light as long as actions are running */

		/* prepare header of possible answer frame */

		switch (((wibocmd_hdr_t*) rxbuf)->command) {

		case WIBOCMDCODE_PING:
			pingrep.hdr.destaddr = ((wibocmd_hdr_t*) rxbuf)->srcaddr;
			pingrep.crc = datacrc;

			trx_reg_write(RG_TRX_STATE, CMD_TX_ARET_ON);
			trx_frame_write(sizeof(wibocmd_pingreply_t) + strlen(BOARD_NAME) + 1 + 2, (uint8_t*)&pingrep);

			TRX_SLPTR_HIGH();
			TRX_SLPTR_LOW();

#if SERIALDEBUG > 0
			puts("Ping");
#endif

#if RADIO_TYPE == RADIO_ATMEGA128RFA1_A ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_B ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_C ||\
      RADIO_TYPE == RADIO_ATMEGA128RFA1_D
			while (!(trx_reg_read(RG_IRQ_STATUS) & TRX_IRQ_TX_END));
			trx_reg_write(RG_IRQ_STATUS, TRX_IRQ_TX_END); /* clear the flag */
#else
			while (!(trx_reg_read(RG_IRQ_STATUS) & TRX_IRQ_TRX_END));
#endif
			trx_reg_write(RG_TRX_STATE, CMD_RX_AACK_ON);
			break;

		case WIBOCMDCODE_RESET:
#if SERIALDEBUG > 0
			puts("Reset");
#endif
			addr = SPM_PAGESIZE;	/* misuse as counter */
			ptr = pagebuf;
			do{
				*ptr++ = 0xFF;
			}while(--addr);
			addr = 0;
			datacrc = 0;
			pagebufidx = 0;
			break;

		case WIBOCMDCODE_DATA:
#if SERIALDEBUG > 0
			printf("Data[%d]", ((wibocmd_data_t*) rxbuf)->dsize);
			for (i = 0; i < ((wibocmd_data_t*) rxbuf)->dsize; i++) {
				printf(" %02X", ((wibocmd_data_t*) rxbuf)->data[i]);
			}
			;
			putchar('\n');
#endif
			ptr = ((wibocmd_data_t*) rxbuf)->data;
			i = ((wibocmd_data_t*) rxbuf)->dsize;
			do {
				datacrc = _crc_ccitt_update(datacrc, *ptr);
				pagebuf[pagebufidx++] = *ptr++;
				if (pagebufidx >= PAGEBUFSIZE) {
					boot_program_page(addr, pagebuf);
					addr += SPM_PAGESIZE;
					pagebufidx = 0;
				}
			} while (--i);
			break;

		case WIBOCMDCODE_FINISH:
#if SERIALDEBUG > 0
			puts("Finish");
#endif
			boot_program_page(addr, pagebuf);
			addr += SPM_PAGESIZE;
			pagebufidx = 0;
			break;

		case WIBOCMDCODE_EXIT:
#if SERIALDEBUG > 0
			puts("Exit");
#endif
			boot_rww_enable();
			app();
			break;

		default:
			/* unknown or unhandled command */
			break;
		};
	}
}

/* EOF */

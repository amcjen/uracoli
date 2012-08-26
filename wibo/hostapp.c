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
#include <avr/io.h>
#include <avr/interrupt.h>

#include <board.h>
#include <ioutil.h>
#include <radio.h>
#include <hif.h>

#include "cmdif.h"
#include "wibohost.h"

#ifndef BAUD			/* try to get baudrate from build configuration settings */
#define BAUD (38400UL)	/* otherwise use this one */
#endif

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

/* setup stream for UART communication */
static FILE usart_stdio = FDEV_SETUP_STREAM(hif_putc_libcwrapper, hif_getc_libcwrapper, _FDEV_SETUP_RW);

int main()
{
	LED_INIT();
	hif_init(BAUD);

	wibohost_init();

	stdin = stdout = stderr = &usart_stdio;

	sei();

	printf_P(PSTR("\n\nWIBO Host <Build %s %s>\n"), __DATE__, __TIME__);
	printf_P(PSTR("CHANNEL=0x%02X, PANID=0x%04X, SHORT_ADDR=0x%04X\n"),
			wibohost_getnodecfg()->channel,
			wibohost_getnodecfg()->pan_id,
			wibohost_getnodecfg()->short_addr
			);

	for (;;) {
		cmdif_task();
	}
}

/* EOF */

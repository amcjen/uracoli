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
 * @brief Command interface for wireless bootloader application
 * Prepared for interfacing with a PC Host + Python
 *
 * @author Daniel Thiele
 *         Dietzsch und Thiele PartG
 *         Bautzner Str. 67 01099 Dresden
 *         www.ib-dt.de daniel.thiele@ib-dt.de
 *
 * @ingroup grpAppWiBo
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <radio.h>
#include <board.h>
#include <hif.h>

#include "cmdif.h"
#include "wibocmd.h"
#include "wibohost.h"
#include "hexparse.h"

/* variable for function cmd_feedhex()
 * placed here (global) to store in SRAM at compile time
 */
static hexrec_t hexrec;

static volatile uint8_t pingreplied = 0;

#define MAXLINELEN (80)
static uint8_t lnbuf[MAXLINELEN + 1];

/*
 * \brief Wait for complete line, no character echoing
 *
 * @return 1 for line completed, 0 else
 */
static inline uint8_t getline()
{
	uint16_t inchar;
	static uint8_t idx = 0;

	inchar = getchar();
	if (inchar < 0x100) {
		lnbuf[idx] = 0x00; /* NULL terminated string */
		if ((inchar == '\n') || (inchar == '\r')) {
			idx = 0;
			return 1;
		} else if (idx < MAXLINELEN) {
			lnbuf[idx++] = inchar;
		} else {
			/* TODO: catch line length overflow */
		}
	}

	return 0;
}

/*
 * \brief Print "OK" to stdout, used to save code space
 */
static inline void printok()
{
	puts_P(PSTR("OK"));
}

/*
 * \brief Called asynchronous when ping reply frame is received
 */
void cb_wibohost_pingreply(wibocmd_pingreply_t *pr, uint8_t lqi, int8_t ed)
{
	printf_P(
			PSTR("OK {'short_addr':0x%02X, 'boardname':'%s', 'swversion':0x%02X, 'crc':0x%04X, 'pagesize':0x%X, 'lqi':%d, 'ed':%d}\n"),
			pr->hdr.srcaddr, pr->boardname, pr->swversion, pr->crc, pr->pagesize, lqi, ed);
	pingreplied = 1;
}

/*
 * \brief Command to execute wibohost_ping() functions
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 */
static inline void cmd_ping(char **params)
{
	volatile uint16_t timeout = 0x7FFF;
	uint16_t short_addr;

	short_addr = strtol(params[0], NULL, 16);
	pingreplied = 0;
	wibohost_ping(short_addr);

	/* now wait for the reply */
	while (!pingreplied && --timeout)
		; /* software timeout */

	if (!timeout) {
		puts_P(PSTR("ERR ping timeout"));
	} else {
		/* do nothing, printout was done by cb_wibohost_pingreply() */
	}
}

/*
 * \brief Command to execute wibohost_finish() functions
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 */
static inline void cmd_finish(char **params)
{
	uint16_t short_addr;

	short_addr = strtol(params[0], NULL, 16);
	wibohost_finish(short_addr);
	printok();
}

/*
 * \brief Command to execute wibohost_feed() functions
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 * Expected parameters
 *  (1) short_addr
 *  (2) line from intel hex-file
 *
 */
static inline void cmd_feedhexline(char **params)
{
	uint16_t short_addr;
	short_addr = strtol(params[0], NULL, 16);
	if (!parsehexline((uint8_t*) params[1], &hexrec)) {
		puts_P(PSTR("ERR parsing hexline"));
	}
	if (HEX_RECTYPE_DATA == hexrec.type) { /* only feed data type lines */
		wibohost_feed(short_addr, hexrec.data, hexrec.len);
		printok();
	} else {
		printf_P(PSTR("ERR ignoring rec type %d\n"), hexrec.type);
	}
}

/*
 * \brief Command to receive a complete hex-file
 *
 * This is intended to use from a serial terminal instead of a python command interface
 *
 * Expected parameters
 *  (1) short_addr
 *
 */
static inline void cmd_feedhexfile(char **params)
{
	uint8_t error=0;
	uint16_t short_addr;

	short_addr = strtol(params[0], NULL, 16);
	puts_P(PSTR("OK upload hex-file now...\n"));

	do{
		/* fetch line */
		do{}while(0 == getline());

		if (!parsehexline((uint8_t*) lnbuf, &hexrec)) {
			puts_P(PSTR("ERR parsing hexline"));
			error=1;
		}

		/* TODO: handling address jump records */

		if (HEX_RECTYPE_DATA == hexrec.type) { /* only feed data type lines */
			wibohost_feed(short_addr, hexrec.data, hexrec.len);
		}else if(HEX_RECTYPE_EOF == hexrec.type) {
			printf_P(PSTR("OK end of file"));
			error=1;
		} else {
			printf_P(PSTR("ERR ignoring rec type %d\n"), hexrec.type);
			error=1;
		}
	}while(0 == error);
}

static inline void cmd_setchannel(char **params)
{
	uint8_t channel;
    channel = strtol(params[0], NULL, 10);
    radio_set_param(RP_CHANNEL(channel));
	printok();
}

/*-       printf_P(PSTR("OK panid=0x%04X shortaddr=0x%04X\n"), cfg->pan_id,

 * \brief Command to execute wibohost_reset() functions
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 */
static inline void cmd_reset(char **params)
{
	wibohost_reset(); /* always reset all nodes */
	printok();
}

/*
 * \brief Command to execute wibohost_exit() functions
 *-       printf_P(PSTR("OK panid=0x%04X shortaddr=0x%04X\n"), cfg->pan_id,

 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 *
 * Expected parameters
 *  (1) short_addr
 *
 */
static inline void cmd_exit(char **params)
{
	uint16_t short_addr;

	short_addr = strtol(params[0], NULL, 16);
	wibohost_exit(short_addr);
	printok();
}

/*
 * \brief Command to execute wibohost_getcrc() functions
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 * Expected Parameters
 * (1) String that is echoed
 */
static inline void cmd_checkcrc(char **params)
{
	printf_P(PSTR("OK 0x%04X\n"), wibohost_getcrc());
}

/*
 * \brief Command to echo a string
 * This is intended to check the UART connection
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 */
static inline void cmd_echo(char **params)
{
	printf_P(PSTR("OK %s\n"), params[0]);
}

/*
 * \brief Command to echo a string
 * This is intended to check the UART connection
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 */
static inline void cmd_info(char **params)
{
	node_config_t *cfg = wibohost_getnodecfg();
	printf_P(PSTR("OK panid=0x%04X shortaddr=0x%04X\n"), cfg->pan_id, 
                    cfg->short_addr);
}

/*
 * \brief Command to jump into bootloader
 * This is used to support an example application where the host does handle the bootloader itself and
 * an example application residing in the application section of the remote node
 *
 * @param params Pointer to list of strings containing the arguments
 *               checking for valid number of arguments is done outside this function
 *
 *
 * Expected parameters
 *  (1) short_addr
 *
 */
static inline void cmd_xmpljbootl(char **params)
{
	uint16_t short_addr;

	short_addr = strtol(params[0], NULL, 16);
	wibohost_xmpljbootl(short_addr);
	printok();
}

/* List of commands that are available at the command interface */
const struct {
	const char *name; /**< Name of the command */
	void (*execfunc)(char**); /**< Function to be called */
	uint8_t nbparams; /**< Expected number of parameters */
	const char *description; /**< Textual description of the command to print help screen */
} commands[] = {
		{ "ping", cmd_ping, 1, "Ping a node" },
		{ "finish", cmd_finish, 1, "Finish a node (force write)" },
		{ "feedhex", cmd_feedhexline, 2, "Feed a line of hex file to a node" },
		{ "feedhexfile", cmd_feedhexfile, 1, "Feed hex file to a node" },
		{ "reset", cmd_reset, 0, "Reset a node" },
		{ "exit", cmd_exit, 1, "Exit node bootloader" },
		{ "crc", cmd_checkcrc, 0, "Get data CRC" },
		{ "echo", cmd_echo, 1, "Echo a string" },
		{ "info", cmd_info, 0, "Information about myself" },
		{ "xmpljbootl", cmd_xmpljbootl, 1, "Example Application, jump into bootloader" },
        { "channel", cmd_setchannel, 1, "Set radio channel" }
        };


/*
 * \brief Parsing a shell input line
 *
 * @param *ln String containing the command line to parse
 */
static inline void process_cmdline(char *ln)
{
	char *params[10];
	uint8_t nbparams;
	uint8_t i;
	uint8_t found=0;

	nbparams = hif_split_args(ln, 10, params);

	for (i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
		if (!strcasecmp(params[0], commands[i].name)) {
			if (commands[i].nbparams >= (nbparams - 1)) { /* substract the command itself */
				commands[i].execfunc(&params[1]);
				found=1;
			} else {
				puts_P(PSTR("ERR Parameter"));
			}
		}
	}

	if(!found){
		puts_P(PSTR("ERR Unknown command"));
	}
}

void cmdif_task(void)
{
	if (getline()) {
		process_cmdline((char*) lnbuf);
	}
}

/* EOF */

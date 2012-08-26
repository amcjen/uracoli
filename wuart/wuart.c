/* Copyright (c) 2007 Axel Wachtler
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
 * @brief Implementation of the @ref grpAppWuart.
 *
 * This Application implements a wireless UART bridge.
 *
 * @ingroup grpAppWuart
 */

#include <string.h>
#include "radio.h"
#include "transceiver.h" /** @todo add a radio function for trx_identify() */
#include "ioutil.h"
#include "timer.h"
#include "prot_wuart.h"

/* === Macros ======================================== */

/** Max. number of payload bytes per frame */
#define PAYLD_SIZE        (PROT_WUART_PAYLD_SIZE)
/** Number of bytes for CRC16 */
#define CRC_SIZE          (sizeof(crc_t))
/** Maximum frame size */
#define UART_FRAME_SIZE   (PROT_WUART_HEADER_SIZE +\
                           PAYLD_SIZE + CRC_SIZE )
/** Index of first payload byte */
#define PAYLD_START       (PROT_WUART_HEADER_SIZE)
/** Index of last payload byte */
#define PAYLD_END         (UART_FRAME_SIZE - CRC_SIZE)
/** END of line delimitter */
#define NL "\n\r"

/** escape state : none / normal data mode */
#define ESC_NONE    (0)

/** escape state : the first timeout had occured */
#define ESC_TMO_1   (1)

/** escape state : the +++ pattern was detected */
#define ESC_PATTERN (2)

/** escape state : the second timeout had occured */
#define ESC_TMO_2   (3)

#ifndef DEFAULT_RADIO_CHANNEL
/** radio channel */
# if defined(TRX_SUPPORTS_BAND_800)
#  define DEFAULT_RADIO_CHANNEL    (0)
# elif defined(TRX_SUPPORTS_BAND_900) && defined(REGION_USA)
#  define DEFAULT_RADIO_CHANNEL    (5)
# elif defined(TRX_SUPPORTS_BAND_2400)
#  define DEFAULT_RADIO_CHANNEL    (17)
# else
#  error "No supported frequency band found"
# endif
#endif

/* === Types ========================================= */

/** Data type for CRC16 values */
typedef uint16_t crc_t;

/** Buffer type */
typedef struct
{
    uint8_t start;
    uint8_t end;
    uint8_t buf[UART_FRAME_SIZE];
} wuart_buffer_t;


/** application states */
typedef enum
{
    /** application is in command mode */
    CMD_MODE,
    /** application is in data mode */
    DATA_MODE,
} wuart_state_t;

/* === globals ====================================== */

node_config_t PROGMEM nc_flash = { .short_addr = 0xBEEF,
                                   .pan_id = 0x2222,
                                   .channel = DEFAULT_RADIO_CHANNEL,
                                 };

static uint16_t rxcnt;
static uint16_t txcnt;

volatile bool chkrxbuf;
volatile bool txpending;
volatile wuart_state_t wuart_state;
bool echo = false;

timer_hdl_t tmr_uart;
wuart_buffer_t rxbuf[2];
volatile uint8_t rxbuf_idx = 0;

node_config_t NodeConfig;

/* === prototypes ================================= */
wuart_state_t wuart_check_escape(uint8_t * cmd, uint8_t size);
wuart_state_t wuart_parse_at_command(char *cmd);

/**
 * @brief Initialize MCU ressources.
 */
void wuart_init()
{
trx_param_t trxp;

    LED_INIT();
    LED_SET_VALUE(0);
    KEY_INIT();
//    hif_init(19200);
    hif_init(9600);
    timer_init();

    rxbuf_idx = 0;
    memset(rxbuf, 0, sizeof(rxbuf));
    rxbuf[0].end = PAYLD_START;
    rxbuf[1].end = PAYLD_START;
    radio_init(rxbuf[rxbuf_idx].buf, UART_FRAME_SIZE);

    sei();
    wuart_state = DATA_MODE;
    radio_set_param(RP_CHANNEL(17));
    radio_set_param(RP_IDLESTATE(STATE_RX));
    trx_parms_get(&trxp);

    /* 1st trial: read from internal EEPROM */
    if (get_node_config_eeprom(&NodeConfig, 0) == 0)
    {
        /* using EEPROM config */;
    }
    /* 2nd trial: read from FLASHEND */
    else if (get_node_config(&NodeConfig) == 0)
    {
        /* using FLASHEND config */;
    }
    /* 3rd trial: read default values compiled into the application */
    else
    {
        /* using application default config */;
        memcpy_P(&NodeConfig, &nc_flash, sizeof(node_config_t) );
    }

    if (((1UL<<NodeConfig.channel) & TRX_SUPPORTED_CHANNELS) == 0)
    {
        /* fall back to DEFAULT_CHANNEL,*/
        PRINTF("Invalid channel: %d use now: %d",
                NodeConfig.channel, DEFAULT_RADIO_CHANNEL);
        NodeConfig.channel = DEFAULT_RADIO_CHANNEL;

    }

    radio_set_param(RP_CHANNEL(NodeConfig.channel));


    PRINTF("Wuart 0.2 chan=%d radio %02x.%02x"NL,
            trxp.chan,
            trx_reg_read(RG_PART_NUM),
            trx_reg_read(RG_VERSION_NUM));

    /*Don't care about the revision, verify if PART_NUM does match. */

    if (trx_identify() & INVALID_PART_NUM)
    {
        PRINTF("radio does not match: expected %d.%d got %d.%d"NL, 
              RADIO_PART_NUM, RADIO_VERSION_NUM, 
              trx_reg_read(RG_PART_NUM),
              trx_reg_read(RG_VERSION_NUM)
              );

    }
}


/**
 * @brief Main function of WUART application
 */
int main(void)
{
uint16_t inchar;

uint8_t txbuf[UART_FRAME_SIZE], cmdidx = 0, inbytes, *cmd, tmp;
wuart_buffer_t *hif_tx_buf;
prot_wuart_header_t *hdr;
time_t lasttxtime = 0, currtime;
bool do_send;

    wuart_init();
    chkrxbuf = true;
    txpending = false;
    cmd = &txbuf[PAYLD_START];
    hdr = (prot_wuart_header_t*)txbuf;
    hdr->fcf = PROT_FCF_DATA;
    hdr->seq = 0;
    hdr->dst = 0xaa;
    hdr->src = 0xbb;
DBG_INIT();

    while(1)
    {

        if (wuart_state == DATA_MODE)
        {
            currtime = timer_systime();
            if (1)
            {

                inbytes = hif_get_blk(cmd+cmdidx, (PAYLD_SIZE-cmdidx));
                cmdidx += inbytes;
                if (cmdidx > 0)
                {
                    cmd[cmdidx]=0;
                }

#if 0
                wuart_state = wuart_check_escape(cmd, cmdidx);
                if (wuart_state == CMD_MODE)
                {
                    cmdidx = 0;
                    radio_set_state(STATE_OFF);
                    PRINT(NL"EXIT DATA MODE"NL);
                    break;
                }
                else
#endif
                do_send = false;
                if (cmdidx == PAYLD_SIZE)
                {
                    do_send = true;
                }
                else if (((currtime - lasttxtime) > 20)  && (cmdidx > 0))
                {
                    do_send = true;
                }
                if ((do_send == true) && (txpending == false))
                {

                    radio_set_state(STATE_TXAUTO);

                    hdr->seq ++;
                    radio_send_frame(PAYLD_START + cmdidx + CRC_SIZE, txbuf, 0);
                    txpending = true;
                    lasttxtime = currtime;
                    cmdidx = 0;
                    tmp = 0;
                    LED_SET(1);

                    if (txpending != true)
                    {
                        hif_puts(NL"TXFAILED"NL);

                    }
                    else
                    {
                        LED_CLR(0);
                    }
                    if (echo)
                    {
                        hif_puts((char*)cmd);
                    }
                }
                else if (cmdidx == 0)
                {
                    /* outa here, make a break for a while */
                    chkrxbuf = false;
                }
            }
        }

        if (wuart_state == CMD_MODE)
        {
            inchar = hif_getc();
            if (inchar < 0x100)
            {
                if (echo)
                {
                    hif_putc(inchar);
                    if (inchar == '\n') hif_putc('\r');
                    if (inchar == '\r') hif_putc('\n');
                }
                if (inchar == '\n' || inchar == '\r')
                {
                    /* terminate string */
                    cmd[cmdidx] = 0;
                    wuart_state = wuart_parse_at_command((char*)cmd);
                    cmdidx = 0;
                    memset(cmd, 0, sizeof(cmd));
                    if (wuart_state == DATA_MODE)
                    {
                        radio_set_state(STATE_RX);
                    }
                }
                else
                {
                    cmd[cmdidx++] = inchar;
                }
            }
        }

        /* uart transmit handler */
        hif_tx_buf = &rxbuf[(rxbuf_idx + 1)&1];
        if (hif_tx_buf->end > PAYLD_START)
        {
            uint8_t *p = &hif_tx_buf->buf[hif_tx_buf->start];
            uint8_t sz = hif_tx_buf->end - hif_tx_buf->start;
            hif_tx_buf->start += hif_put_blk(p, sz);
            if (hif_tx_buf->start >= hif_tx_buf->end)
            {
                hif_tx_buf->end = PAYLD_START;
            }
        }
    }
    return 0;
}


/**
 * Implementation of callback function @ref usr_radio_tx_done.
 */
#ifdef DOXYGEN
uint8_t * wuart_usr_radio_tx_done()
#else
void usr_radio_tx_done(radio_tx_done_t status)
#endif
{
    if (status == TX_OK)
    {
        LED_CLR(1);
        txcnt ++;
    }
    txpending = false;
}




static uint8_t decode_154headerlen(uint8_t *frm)
{
uint8_t ret = 3;
    if (frm[1] & 8)
    {
        ret += 4;
    }
    if (frm[1] & 4)
    {
        ret += 6;
    }
    if (frm[1] & 0x80)
    {
        ret += 4;
    }
    if (frm[1] & 0x40)
    {
        ret += 6;
    }
    if ((frm[1] & 0xc0) != 0 && (frm[0] & 0x40) != 0)
    {
        ret -= 2;
    }
    return ret;

}

/**
 * Implementation of callback function @ref usr_radio_receive_frame.
 */
#ifdef DOXYGEN
uint8_t * wuart_usr_radio_receive_frame()
#else
uint8_t * usr_radio_receive_frame(uint8_t len, uint8_t *frm, uint8_t lqi,
                                  int8_t ed, uint8_t crc)
#endif
{
static uint8_t rxseq = 0;
static uint16_t  seqerr;
prot_wuart_header_t *rxhdr;
    uint8_t __sreg = SREG; cli();
    LED_TOGGLE(1);
    if (crc == 0)
    {
        if( rxbuf[(rxbuf_idx^1)].end == PAYLD_START)
        {
            /* yes, we can do a swap */
            rxhdr = (prot_wuart_header_t*) frm;
            if (rxhdr->seq != rxseq)
            {
                seqerr++;
            }
            rxseq = rxhdr->seq+1;
            rxbuf[rxbuf_idx].start = decode_154headerlen(frm);
            rxbuf[rxbuf_idx].end = len - CRC_SIZE;
            rxbuf_idx ^= 1;
            frm = rxbuf[rxbuf_idx].buf;
        }
        rxcnt ++;
    }
    SREG = __sreg;
    return frm;
}

/**
 * check for the sequence for escape from the data mode
 *
 * <pre>
 *   [ no data on HIF ]+++[ no data on HIF ]
 *         |            |       |
 *         |            |       `- ESC_TMO_2
 *         |            `- ESC_PATTERN
 *         `- ESC_TMO_1
 * </pre>
 *
 *  @dot
 *    digraph "wuart_escape" {
 *        rankdir=LR
 *
 *        ESC_NONE -> ESC_TMO_1 -> ESC_PATTERN -> ESC_TMO_2
 *        ESC_PATTERN ->  ESC_NONE [label = "!+"]
 *    }
 *  @enddot
 *
 */
wuart_state_t wuart_check_escape(uint8_t * cmd, uint8_t size)
{

static uint8_t  pattern, tmo, esc_state, i;
wuart_state_t ret = DATA_MODE;


    if (size > 0)
    {
        /* check pattern for occurence of +++ */
#if 0
        if (size > 3)
        {
            /* more data received */
            esc_state = ESC_NONE;
            tmo = 0;
            pattern = 0;
        }
        else
#endif
        {
            /* there could be the +++ sequence in */
            if (esc_state > ESC_NONE)
            {
                for(i=0; i<size; i++)
                {
                    if (cmd[i] == '+')
                    {
                        pattern++;
                    }
                    else
                    {
                        pattern = 0;
                        esc_state = ESC_NONE;
                    }
                    if (pattern == 3)
                    {
                        esc_state = ESC_PATTERN;
                        tmo = 0;
                    }
                }
            }
        }
    }
    else
    {
        /* counting the occurences of empty interface */
        tmo += 1;
        if (tmo  > 10)
        {
            esc_state = (esc_state == ESC_PATTERN) ? ESC_TMO_2 : ESC_TMO_1;
            tmo = 0;
        }
    }
    ret = (esc_state != ESC_TMO_2) ? DATA_MODE : CMD_MODE;
    if (ret == CMD_MODE)
    {
        esc_state = ESC_NONE;
        tmo = 0;
        pattern = 0;
    }
    return ret;
}


/**
 * Process AT command
 */
wuart_state_t wuart_parse_at_command(char *cmd)
{

    wuart_state_t ret = CMD_MODE;
    bool atok;
    if (cmd[0] == 'A' && cmd[1] == 'T')
    {
        cmd += 2;
        while (*cmd != 0)
        {

            atok = true;
            if (*cmd == 'D')
            {
                PRINT("ENTER DATA MODE"NL);
                ret = DATA_MODE;
            }
            else if (*cmd == 'E')
            {
                cmd++;
                if (*cmd == '0')
                {
                    echo = false;
                }
                else if (*cmd == '1')
                {
                    echo = true;

                }
                else if (*cmd == '?')
                {
                    PRINTF("ECHO %s"NL, echo ? "ON":"OFF");
                }
                else
                {
                    atok = false;
                }
            }
            else if (*cmd == 'I')
            {
                trx_param_t trxp;
                trx_parms_get(&trxp);
                PRINTF( "E%d "\
                        "CHA:%d TXP:%d "\
                        "CCA:%d EDT:%d "\
                        "CLK:%d"NL,
                    echo,
                    trxp.chan, trxp.txp, trxp.cca,
                    trxp.edt, trxp.clkm);
            }
            else if (*cmd == 'S')
            {
                uint8_t reg;
                cmd++;
                reg = 0;
                while (*cmd != '=' && *cmd != '?')
                {
                    reg *= 10;
                    reg += (*cmd++ - '0');
                }
                if (*cmd == '=')
                {
                    uint8_t val;
                    cmd ++;
                    val = 0;
                    while(*cmd >= '0' && *cmd <= '9' )
                    {
                        val *= 10;
                        val += (*cmd++ - '0');
                    }
                    PRINTF("S%d = %d"NL, reg, val);
                }
                else if (*cmd == '?')
                {
                    PRINTF("S%d = %d"NL, reg, 42);
                }
                else
                {
                    cmd ++;
                    atok = false;
                }

            }
            else
            {
                atok = false;
            }
            cmd ++;
            PRINTF("%s"NL, atok ? "OK" : "ERROR");
        }
    }
    return ret;
}

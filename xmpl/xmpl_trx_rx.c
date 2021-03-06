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
/* Example for receiving frames */

#include "board.h"
#include "transceiver.h"
#include "ioutil.h"
#include  <util/crc16.h>
#include "xmpl.h"


static uint8_t rxfrm[MAX_FRAME_SIZE];
static volatile uint8_t rxcnt;
int main(void)
{
trx_regval_t rval;

    /* This will stop the application before initializing the radio transceiver
     * (ISP issue with MISO pin, see FAQ)
     */
    trap_if_key_pressed();

    /* Step 0: init MCU peripherals */
    LED_INIT();
    trx_io_init(SPI_RATE_1_2);
    LED_SET_VALUE(LED_MAX_VALUE);
    LED_SET_VALUE(0);

    /* Step 1: initialize the transceiver */
    TRX_RESET_LOW();
    TRX_SLPTR_LOW();
    DELAY_US(TRX_RESET_TIME_US);
    TRX_RESET_HIGH();
    trx_reg_write(RG_TRX_STATE,CMD_TRX_OFF);
    DELAY_US(TRX_INIT_TIME_US);
    rval = trx_bit_read(SR_TRX_STATUS);
    ERR_CHECK(TRX_OFF!=rval);
    LED_SET_VALUE(1);

    /* Step 2: setup transmitter
     * - configure radio channel
     * - go into RX state,
     * - enable "receive end" IRQ
     */
    trx_bit_write(SR_CHANNEL,CHANNEL);
    trx_reg_write(RG_TRX_STATE,CMD_RX_ON);
#if defined(TRX_IRQ_TRX_END)
    trx_reg_write(RG_IRQ_MASK,TRX_IRQ_TRX_END);
#elif defined(TRX_IRQ_RX_END)
    trx_reg_write(RG_IRQ_MASK,TRX_IRQ_RX_END);
#else
#  error "Unknown IRQ bits"
#endif
    sei();
    LED_SET_VALUE(2);

    /* Step 3: Going to receive frames */
    rxcnt = 0;

    LED_SET_VALUE(0);
    while(1);
}

#if defined(TRX_IF_RFA1)
ISR(TRX24_RX_END_vect)
{
uint8_t flen, *pfrm, tmp;
uint16_t crc;

    /* upload frame and check for CRC16 validity */
    pfrm = rxfrm;
    flen = trx_frame_read(pfrm, sizeof(rxfrm), NULL);
    crc = 0;
    do
    {
        crc = _crc_ccitt_update(crc, *pfrm++);
    }
    while(flen--);
    /* if crc is correct, update RX frame counter */
    if (crc == 0)
    {
        rxcnt ++;
    }
    /* display current rx statistics
     * LED[0] toggles with every received frame
     * LED[1:n] display the count of frames received with valid CRC
     */
    tmp = (rxcnt<<1) | ((LED_GET_VALUE()&1)^1);
    LED_SET_VALUE(tmp);
}
#else  /* !RFA1 */
ISR(TRX_IRQ_vect)
{
static volatile trx_regval_t irq_cause;
uint8_t flen, *pfrm, tmp;
uint16_t crc;

    irq_cause = trx_reg_read(RG_IRQ_STATUS);
    if (irq_cause & TRX_IRQ_TRX_END)
    {
        /* upload frame and check for CRC16 validity */
        pfrm = rxfrm;
        flen = trx_frame_read(pfrm, sizeof(rxfrm), NULL);
        crc = 0;
        do
        {
            crc = _crc_ccitt_update(crc, *pfrm++);
        }
        while(flen--);
        /* if crc is correct, update RX frame counter */
        if (crc == 0)
        {
            rxcnt ++;
        }
        /* display current rx statistics
         * LED[0] toggles with every received frame
         * LED[1:n] display the count of frames received with valid CRC
         */
        tmp = (rxcnt<<1) | ((LED_GET_VALUE()&1)^1);
        LED_SET_VALUE(tmp);
    }
}
#endif  /* RFA1 */

/* EOF */

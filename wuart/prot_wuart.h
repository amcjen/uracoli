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
 * @brief Simple peer to peer protocoll definition for wireless UART
 *
 * @ingroup grpAppProt
 */
#ifndef PROT_WUART_H
#define PROT_WUART_H

#include <stdint.h>

/* === Types =========================================== */

/** Frame header structure
 *
 * This frame format uses the same frame format like 802.15.4 frames,
 * but the interpretation of FCF bits for adressing uses reserved values.
 *
 */
typedef struct
{
    uint16_t fcf;   /**< Frame control field*/
    uint8_t  seq;   /**< Sequence number */
    uint16_t dst;   /**< 16 bit destination address */
    uint16_t src;   /**< 16 bit source address */
} prot_wuart_header_t;

/* === Macros ========================================== */

#define FCF_DATA_FRAME (1)
#define FCF_ACK_FRAME  (2)
#define FCF_ACK_REQ    (_BV(5))
/* 1 is is a reserved addressing field  contents in 15.4,
   we use to indicate, that only a 16 bit address is present */
#define FCF_DST_ADDR_01 (_BV(11))
#define FCF_SRC_ADDR_01 (_BV(15))

//#define PROT_WUART_PAYLD_SIZE   (32)
#define PROT_WUART_PAYLD_SIZE   (118)
#define PROT_WUART_HEADER_SIZE (sizeof(prot_wuart_header_t))
#define PROT_FCF_DATA (FCTL_DATA | FCTL_DST_SHORT)
#define PROT_FCF_ACK (FCF_ACK_FRAME)

/* === Prototypes ====================================== */

#endif

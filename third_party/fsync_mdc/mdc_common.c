/*-
 * mdc_common.c — CRC helpers for MDC1200 decoder.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2011. GPL-2.0.
 * Included textually by mdc_decode.c — do not compile separately.
 */

static mdc_u16_t _flip(mdc_u16_t crc, mdc_int_t bitnum)
{
    mdc_u16_t crcout = 0, i, j;
    j = 1;
    for (i = 1 << (bitnum - 1); i; i >>= 1) {
        if (crc & i) crcout |= j;
        j <<= 1;
    }
    return crcout;
}

static mdc_u16_t _docrc(mdc_u8_t *p, int len)
{
    mdc_int_t i, j;
    mdc_u16_t c;
    mdc_int_t bit;
    mdc_u16_t crc = 0x0000;

    for (i = 0; i < len; i++) {
        c = (mdc_u16_t)*p++;
        c = _flip(c, 8);
        for (j = 0x80; j; j >>= 1) {
            bit = crc & 0x8000;
            crc <<= 1;
            if (c & j) bit ^= 0x8000;
            if (bit)   crc ^= 0x1021;
        }
    }

    crc = _flip(crc, 16);
    crc ^= 0xffff;
    crc &= 0xFFFF;
    return crc;
}

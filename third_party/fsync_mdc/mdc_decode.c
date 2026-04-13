/*-
 * mdc_decode.c — Motorola MDC1200 1200 bps XOR-precoded MSK decoder.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2005, 2010. GPL-2.0.
 * Adapted for Bazel/C++17 build by sdrmon project.
 */

#include <stdlib.h>
#include "mdc_decode.h"
#include "mdc_common.c"   /* textual include */

#ifndef TWOPI
#define TWOPI (2.0 * 3.1415926535)
#endif

mdc_decoder_t *mdc_decoder_new(int sampleRate)
{
    mdc_decoder_t *decoder;
    mdc_int_t i;

    decoder = (mdc_decoder_t *)malloc(sizeof(mdc_decoder_t));
    if (!decoder) return (mdc_decoder_t *)0L;

    if      (sampleRate == 8000)  decoder->incru = 644245094U;
    else if (sampleRate == 16000) decoder->incru = 322122547U;
    else if (sampleRate == 22050) decoder->incru = 233739716U;
    else if (sampleRate == 32000) decoder->incru = 161061274U;
    else if (sampleRate == 44100) decoder->incru = 116869858U;
    else if (sampleRate == 48000) decoder->incru = 107374182U;
    else    decoder->incru = (mdc_u32_t)(1200 * 2 * ((unsigned long)0x80000000 / sampleRate));

    decoder->good     = 0;
    decoder->indouble = 0;
    decoder->level    = 0;

    for (i = 0; i < MDC_ND; i++) {
        decoder->du[i].thu     = (mdc_u32_t)(i * 2 * ((unsigned long)0x80000000 / MDC_ND));
        decoder->du[i].xorb    = 0;
        decoder->du[i].invert  = 0;
        decoder->du[i].shstate = -1;
        decoder->du[i].shcount = 0;
#ifdef MDC_FOURPOINT
        decoder->du[i].nlstep  = i;
#endif
    }

    decoder->callback         = (mdc_decoder_callback_t)0L;
    decoder->callback_context = (void *)0L;
    return decoder;
}

static void _clearbits(mdc_decoder_t *decoder, mdc_int_t x)
{
    mdc_int_t i;
    for (i = 0; i < 112; i++) decoder->du[x].bits[i] = 0;
}

#ifdef MDC_ECC
static void _gofix(unsigned char *data)
{
    int i, j, b, k;
    int csr[7];
    int syn;
    int fixi, fixj;
    int ec;

    syn = 0;
    for (i = 0; i < 7; i++) csr[i] = 0;

    for (i = 0; i < 7; i++) {
        for (j = 0; j <= 7; j++) {
            for (k = 6; k > 0; k--) csr[k] = csr[k-1];
            csr[0] = (data[i] >> j) & 0x01;
            b = csr[0] + csr[2] + csr[5] + csr[6];
            syn <<= 1;
            if ((b & 0x01) ^ ((data[i+7] >> j) & 0x01)) syn |= 1;
            ec = 0;
            if (syn & 0x80) ++ec;
            if (syn & 0x20) ++ec;
            if (syn & 0x04) ++ec;
            if (syn & 0x02) ++ec;
            if (ec >= 3) {
                syn ^= 0xa6;
                fixi = i; fixj = j - 7;
                if (fixj < 0) { --fixi; fixj += 8; }
                if (fixi >= 0) data[fixi] ^= 1 << fixj;
            }
        }
    }
}
#endif /* MDC_ECC */

static void _procbits(mdc_decoder_t *decoder, int x)
{
    mdc_int_t lbits[112];
    mdc_int_t lbc = 0;
    mdc_int_t i, j, k;
    mdc_u8_t  data[14];
    mdc_u16_t ccrc, rcrc;

    for (i = 0; i < 16; i++)
        for (j = 0; j < 7; j++) {
            k = (j * 16) + i;
            lbits[lbc++] = decoder->du[x].bits[k];
        }

    for (i = 0; i < 14; i++) {
        data[i] = 0;
        for (j = 0; j < 8; j++) {
            k = (i * 8) + j;
            if (lbits[k]) data[i] |= 1 << j;
        }
    }

#ifdef MDC_ECC
    _gofix(data);
#endif

    ccrc = _docrc(data, 4);
    rcrc = (mdc_u16_t)((data[5] << 8) | data[4]);

    if (ccrc == rcrc) {
        if (decoder->du[x].shstate == 2) {
            decoder->extra0 = data[0];
            decoder->extra1 = data[1];
            decoder->extra2 = data[2];
            decoder->extra3 = data[3];
            for (k = 0; k < MDC_ND; k++) decoder->du[k].shstate = -1;
            decoder->good     = 2;
            decoder->indouble = 0;
        } else {
            if (!decoder->indouble) {
                decoder->good   = 1;
                decoder->op     = data[0];
                decoder->arg    = data[1];
                decoder->unitID = (mdc_u16_t)((data[2] << 8) | data[3]);

                switch (data[0]) {
                case 0x35: case 0x55:
                    decoder->good     = 0;
                    decoder->indouble = 1;
                    decoder->du[x].shstate = 2;
                    decoder->du[x].shcount = 0;
                    _clearbits(decoder, x);
                    break;
                default:
                    for (k = 0; k < MDC_ND; k++) decoder->du[k].shstate = -1;
                    break;
                }
            } else {
                decoder->du[x].shstate = 2;
                decoder->du[x].shcount = 0;
                _clearbits(decoder, x);
            }
        }
    } else {
        decoder->du[x].shstate = -1;
    }

    if (decoder->good) {
        if (decoder->callback) {
            (decoder->callback)(
                (int)decoder->good,
                decoder->op, decoder->arg, decoder->unitID,
                decoder->extra0, decoder->extra1,
                decoder->extra2, decoder->extra3,
                decoder->callback_context);
            decoder->good = 0;
        }
    }
}

static int _onebits_mdc(mdc_u32_t n)
{
    int i = 0;
    while (n) { ++i; n &= (n-1); }
    return i;
}

static void _shiftin_mdc(mdc_decoder_t *decoder, int x)
{
    int bit = decoder->du[x].xorb;
    int gcount;

    switch (decoder->du[x].shstate) {
    case -1:
        decoder->du[x].synchigh = 0;
        decoder->du[x].synclow  = 0;
        decoder->du[x].shstate  = 0;
        /* fall through */
    case 0:
        decoder->du[x].synchigh <<= 1;
        if (decoder->du[x].synclow & 0x80000000)
            decoder->du[x].synchigh |= 1;
        decoder->du[x].synclow <<= 1;
        if (bit) decoder->du[x].synclow |= 1;

        gcount  = _onebits_mdc(0x000000ffU & (0x00000007U ^ decoder->du[x].synchigh));
        gcount += _onebits_mdc(0x092a446fU ^ decoder->du[x].synclow);

        if (gcount <= MDC_GDTHRESH) {
            decoder->du[x].shstate = 1;
            decoder->du[x].shcount = 0;
            _clearbits(decoder, x);
        } else if (gcount >= (40 - MDC_GDTHRESH)) {
            decoder->du[x].shstate = 1;
            decoder->du[x].shcount = 0;
            decoder->du[x].xorb    = !(decoder->du[x].xorb);
            decoder->du[x].invert  = !(decoder->du[x].invert);
            _clearbits(decoder, x);
        }
        return;
    case 1: case 2:
        decoder->du[x].bits[decoder->du[x].shcount] = bit;
        decoder->du[x].shcount++;
        if (decoder->du[x].shcount > 111)
            _procbits(decoder, x);
        return;
    default:
        return;
    }
}

#ifdef MDC_FOURPOINT
static void _nlproc(mdc_decoder_t *decoder, int x)
{
    mdc_float_t vnow, vpast;

    switch (decoder->du[x].nlstep) {
    case 3:
        vnow  = (-0.60 * decoder->du[x].nlevel[3]) + (0.97 * decoder->du[x].nlevel[1]);
        vpast = (-0.60 * decoder->du[x].nlevel[7]) + (0.97 * decoder->du[x].nlevel[9]);
        break;
    case 8:
        vnow  = (-0.60 * decoder->du[x].nlevel[8]) + (0.97 * decoder->du[x].nlevel[6]);
        vpast = (-0.60 * decoder->du[x].nlevel[2]) + (0.97 * decoder->du[x].nlevel[4]);
        break;
    default:
        return;
    }

    decoder->du[x].xorb = (vnow > vpast) ? 1 : 0;
    if (decoder->du[x].invert) decoder->du[x].xorb = !(decoder->du[x].xorb);
    _shiftin_mdc(decoder, x);
}
#endif /* MDC_FOURPOINT */

int mdc_decoder_process_samples(mdc_decoder_t *decoder,
                                  mdc_sample_t  *samples,
                                  int numSamples)
{
    mdc_int_t  i, j;
    mdc_sample_t sample;
    mdc_float_t  value;

    if (!decoder) return -1;

    for (i = 0; i < numSamples; i++) {
        sample = samples[i];

#if defined(MDC_SAMPLE_FORMAT_U8)
        value = (((mdc_float_t)sample) - 128.0) / 256.0;
#elif defined(MDC_SAMPLE_FORMAT_U16)
        value = (((mdc_float_t)sample) - 32768.0) / 65536.0;
#elif defined(MDC_SAMPLE_FORMAT_S16)
        value = ((mdc_float_t)sample) / 65536.0;
#elif defined(MDC_SAMPLE_FORMAT_FLOAT)
        value = sample;
#else
#error "no known sample format set"
#endif

#ifdef MDC_FOURPOINT
        for (j = 0; j < MDC_ND; j++) {
            mdc_u32_t lthu = decoder->du[j].thu;
            decoder->du[j].thu += 5 * decoder->incru;
            if (decoder->du[j].thu < lthu) {   /* wrapped */
                decoder->du[j].nlstep++;
                if (decoder->du[j].nlstep > 9) decoder->du[j].nlstep = 0;
                decoder->du[j].nlevel[decoder->du[j].nlstep] = value;
                _nlproc(decoder, j);
            }
        }
#elif defined(MDC_ONEPOINT)
        for (j = 0; j < MDC_ND; j++) {
            mdc_u32_t lthu = decoder->du[j].thu;
            decoder->du[j].thu += decoder->incru;
            if (decoder->du[j].thu < lthu) {
                decoder->du[j].xorb = (value > 0) ? 1 : 0;
                if (decoder->du[j].invert) decoder->du[j].xorb = !(decoder->du[j].xorb);
                _shiftin_mdc(decoder, j);
            }
        }
#else
#error "no decode strategy chosen"
#endif
    }

    if (decoder->good) return decoder->good;
    return 0;
}

int mdc_decoder_get_packet(mdc_decoder_t *decoder,
                            unsigned char *op,
                            unsigned char *arg,
                            unsigned short *unitID)
{
    if (!decoder || decoder->good != 1) return -1;
    if (op)     *op     = decoder->op;
    if (arg)    *arg    = decoder->arg;
    if (unitID) *unitID = decoder->unitID;
    decoder->good = 0;
    return 0;
}

int mdc_decoder_get_double_packet(mdc_decoder_t *decoder,
                                   unsigned char *op,
                                   unsigned char *arg,
                                   unsigned short *unitID,
                                   unsigned char *extra0,
                                   unsigned char *extra1,
                                   unsigned char *extra2,
                                   unsigned char *extra3)
{
    if (!decoder || decoder->good != 2) return -1;
    if (op)     *op     = decoder->op;
    if (arg)    *arg    = decoder->arg;
    if (unitID) *unitID = decoder->unitID;
    if (extra0) *extra0 = decoder->extra0;
    if (extra1) *extra1 = decoder->extra1;
    if (extra2) *extra2 = decoder->extra2;
    if (extra3) *extra3 = decoder->extra3;
    decoder->good = 0;
    return 0;
}

int mdc_decoder_set_callback(mdc_decoder_t *decoder,
                              mdc_decoder_callback_t callbackFunction,
                              void *context)
{
    if (!decoder) return -1;
    decoder->callback         = callbackFunction;
    decoder->callback_context = context;
    return 0;
}

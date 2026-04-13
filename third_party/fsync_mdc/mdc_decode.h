/*-
 * mdc_decode.h — header for the Motorola MDC1200 decoder.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2005, 2010. GPL-2.0.
 */

#ifndef _MDC_DECODE_H_
#define _MDC_DECODE_H_

#include "mdc_types.h"

#define MDC_GDTHRESH 5
#define MDC_ECC
#define MDC_FOURPOINT   /* recommended four-point method */

#ifdef MDC_FOURPOINT
#define MDC_ND 5
#endif

#ifdef MDC_ONEPOINT
#define MDC_ND 4
#endif

typedef void (*mdc_decoder_callback_t)(
    int frameCount,
    unsigned char op,
    unsigned char arg,
    unsigned short unitID,
    unsigned char extra0,
    unsigned char extra1,
    unsigned char extra2,
    unsigned char extra3,
    void *context);

typedef struct {
    mdc_u32_t  thu;
    mdc_int_t  xorb;
    mdc_int_t  invert;
#ifdef MDC_FOURPOINT
    mdc_int_t   nlstep;
    mdc_float_t nlevel[10];
#endif
    mdc_u32_t  synclow;
    mdc_u32_t  synchigh;
    mdc_int_t  shstate;
    mdc_int_t  shcount;
    mdc_int_t  bits[112];
} mdc_decode_unit_t;

typedef struct {
    mdc_decode_unit_t du[MDC_ND];
    mdc_u32_t  incru;
    mdc_int_t  level;
    mdc_int_t  good;
    mdc_int_t  indouble;
    mdc_u8_t   op;
    mdc_u8_t   arg;
    mdc_u16_t  unitID;
    mdc_u8_t   extra0;
    mdc_u8_t   extra1;
    mdc_u8_t   extra2;
    mdc_u8_t   extra3;
    mdc_decoder_callback_t callback;
    void      *callback_context;
} mdc_decoder_t;

mdc_decoder_t *mdc_decoder_new(int sampleRate);

int mdc_decoder_process_samples(mdc_decoder_t *decoder,
                                 mdc_sample_t  *samples,
                                 int numSamples);

int mdc_decoder_get_packet(mdc_decoder_t *decoder,
                            unsigned char *op,
                            unsigned char *arg,
                            unsigned short *unitID);

int mdc_decoder_get_double_packet(mdc_decoder_t *decoder,
                                   unsigned char *op,
                                   unsigned char *arg,
                                   unsigned short *unitID,
                                   unsigned char *extra0,
                                   unsigned char *extra1,
                                   unsigned char *extra2,
                                   unsigned char *extra3);

int mdc_decoder_set_callback(mdc_decoder_t *decoder,
                              mdc_decoder_callback_t callbackFunction,
                              void *context);

#endif /* _MDC_DECODE_H_ */

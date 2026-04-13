/*-
 * fsync_decode.h — header for the Kenwood FleetSync I/II decoder.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2012-2014. GPL-2.0.
 * Extended with GPS decode and JSON output by jfdupuis.
 */

#ifndef _FSYNC_DECODE_H_
#define _FSYNC_DECODE_H_

#include <string>
#include "fsync_types.h"

#ifndef TWOPI
#define TWOPI (2.0 * 3.1415926535)
#endif

#define FSYNC_ND      10   /* total number of decoder phases */
#define FSYNC_ND_12    5   /* phases used for 1200 bps */
#define FSYNC_GDTHRESH 3   /* good-bits threshold */

#define DIFFERENTIATOR /* use differentiator-based zero-crossing */

typedef void (*fsync_decoder_callback_t)(
    int cmd,
    int subcmd,
    int from_fleet,
    int from_unit,
    int to_fleet,
    int to_unit,
    int allflag,
    unsigned char *payload,
    int payload_len,
    unsigned char *raw_msg,
    int raw_msg_len,
    void *context,
    int is_fsync2,
    int is_2400);

typedef struct {
    fsync_float_t hyst;
    fsync_float_t incr;
    fsync_float_t th[FSYNC_ND];
    fsync_float_t thx[FSYNC_ND];
    fsync_int_t   level;
    fsync_float_t lastvalue;
    fsync_float_t accum0[FSYNC_ND];
    fsync_float_t accum1[FSYNC_ND];
    fsync_float_t accum0i[FSYNC_ND];
    fsync_float_t accum1i[FSYNC_ND];
    fsync_int_t   zc[FSYNC_ND];
    fsync_int_t   xorb[FSYNC_ND];
    fsync_u32_t   synclow[FSYNC_ND];
    fsync_u32_t   synchigh[FSYNC_ND];
    fsync_int_t   shstate[FSYNC_ND];
    fsync_int_t   shcount[FSYNC_ND];
    fsync_int_t   fs2state[FSYNC_ND];
    fsync_int_t   fs2w1[FSYNC_ND];
    fsync_int_t   fs2w2[FSYNC_ND];
    fsync_int_t   is_fs2[FSYNC_ND];
    fsync_u32_t   word1[FSYNC_ND];
    fsync_u32_t   word2[FSYNC_ND];
    fsync_u8_t    message[FSYNC_ND][1536];
    fsync_int_t   msglen[FSYNC_ND];
    fsync_int_t   actives;
    fsync_decoder_callback_t callback;
    void         *callback_context;
} fsync_decoder_t;

std::string binary_to_hex(const fsync_u8_t *data, size_t len);

fsync_decoder_t *fsync_decoder_new(int sampleRate);

int fsync_decoder_process_samples(fsync_decoder_t *decoder,
                                   fsync_sample_t *samples,
                                   int numSamples);

int fsync_decoder_end_samples(fsync_decoder_t *decoder);

int fsync_decoder_set_callback(fsync_decoder_t *decoder,
                                fsync_decoder_callback_t callback_function,
                                void *context);

#endif /* _FSYNC_DECODE_H_ */

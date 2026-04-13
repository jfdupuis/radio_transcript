/*-
 * fsync_decode.cc — Kenwood FleetSync I/II 1200/2400 bps MSK decoder.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2012-2014. GPL-2.0.
 * Extended with GPS decode by jfdupuis.
 * Adapted for Bazel/C++17 build by sdrmon project.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "fsync_decode.h"
#include "fsync_common.c"  // textual include — not compiled separately

std::string binary_to_hex(const fsync_u8_t *data, size_t len)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i)
        oss << std::setw(2) << static_cast<unsigned>(data[i]);
    return oss.str();
}

fsync_decoder_t *fsync_decoder_new(int sampleRate)
{
    fsync_decoder_t *decoder;
    fsync_int_t i;

    decoder = (fsync_decoder_t *)malloc(sizeof(fsync_decoder_t));
    if (!decoder)
        return (fsync_decoder_t *)0L;

    decoder->hyst    = 3.0 / 256.0;
    decoder->incr    = (1200.0 * TWOPI) / ((fsync_float_t)sampleRate);
    decoder->actives = 0;
    decoder->level   = 0;
    decoder->lastvalue = 0.0;

    for (i = 0; i < FSYNC_ND; i++) {
        decoder->th[i]  = 0.0 + (((fsync_float_t)i) * (TWOPI / (fsync_float_t)FSYNC_ND_12));
        while (decoder->th[i] >= TWOPI)
            decoder->th[i] -= TWOPI;
        decoder->thx[i]     = decoder->th[i];
        decoder->accum0[i]  = 0.0;
        decoder->accum1[i]  = 0.0;
        decoder->accum0i[i] = 0.0;
        decoder->accum1i[i] = 0.0;
        decoder->zc[i]      = 0;
        decoder->xorb[i]    = 0;
        decoder->shstate[i] = 0;
        decoder->shcount[i] = 0;
        decoder->fs2state[i]= 0;
        decoder->msglen[i]  = 0;
        decoder->is_fs2[i]  = 0;
        decoder->synclow[i] = 0;
        decoder->synchigh[i]= 0;
        decoder->word1[i]   = 0;
        decoder->word2[i]   = 0;
        decoder->fs2w1[i]   = 0;
        decoder->fs2w2[i]   = 0;
    }

    decoder->callback         = (fsync_decoder_callback_t)0L;
    decoder->callback_context = nullptr;
    return decoder;
}

static void _dispatch(fsync_decoder_t *decoder, int x)
{
    unsigned char payload[128];
    int paylen  = 0;
    int aflag, fleetflag;
    fsync_u8_t m0, m1;
    fsync_u8_t *msg      = decoder->message[x];
    fsync_int_t msglen   = decoder->msglen[x];
    fsync_int_t from_fleet, from_unit, to_fleet, to_unit;

    if (msg[0] & 0x01) aflag = 1; else aflag = 0;
    if (msg[1] & 0x01) fleetflag = 1; else fleetflag = 0;

    m0 = msg[0] & 0xfe;
    m1 = msg[1] & 0xfe;

    from_fleet = msg[2] + 99;
    from_unit  = (msg[3] << 4) + ((msg[4] >> 4) & 0x0f) + 999;
    to_unit    = ((msg[4] << 8) & 0xf00) + msg[5] + 999;

    if (fleetflag) {
        if (msglen < 7) return;
        to_fleet = msg[6] + 99;
    } else {
        to_fleet = from_fleet;
    }

    if (from_fleet == 99)  from_fleet = -1;
    if (to_fleet   == 99)  to_fleet   = -1;
    if (to_unit    == 999) to_unit    = -1;
    if (from_unit  == 999) from_unit  = -1;

    if (m1 == 0x42) {
        if (msglen < 11) return;
        paylen = (msg[9] << 8) + msg[10];
        for (int i = 0; i < paylen; i++) {
            int offset = 11 + (6 * ((i / 6) + 1)) - (i % 6);
            if (offset >= msglen)                 return;
            if (i >= (int)sizeof(payload))        return;
            payload[i] = msg[offset];
        }
    } else {
        paylen = 0;
    }

    if (decoder->callback)
        (decoder->callback)(
            (int)m1, (int)m0,
            (int)from_fleet, (int)from_unit,
            (int)to_fleet,   (int)to_unit,
            (int)aflag,
            payload, paylen,
            (unsigned char *)msg, (int)msglen,
            decoder->callback_context,
            decoder->is_fs2[x],
            x < FSYNC_ND_12 ? 0 : 1);
}

static void _procbits(fsync_decoder_t *decoder, int x)
{
    int crc = _fsync_crc(decoder->word1[x], decoder->word2[x]);

    if (crc == (int)(decoder->word2[x] & 0x0000ffff)) {
        decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 24) & 0xff;
        decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >> 16) & 0xff;
        decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >>  8) & 0xff;
        decoder->message[x][decoder->msglen[x]++] = (decoder->word1[x] >>  0) & 0xff;
        decoder->message[x][decoder->msglen[x]++] = (decoder->word2[x] >> 24) & 0xff;
        decoder->message[x][decoder->msglen[x]++] = (decoder->word2[x] >> 16) & 0xff;
        decoder->is_fs2[x] = 0;
        decoder->shstate[x] = 1;
        decoder->shcount[x] = 32;
    } else {
        int ec1 = _fsync2_ecc_repair((decoder->word1[x]) & 0x0000ffff);
        int ec2 = _fsync2_ecc_repair((decoder->word1[x] >> 16) & 0x0000ffff);
        int ec3 = _fsync2_ecc_repair((decoder->word2[x]) & 0x0000ffff);
        int ec4 = _fsync2_ecc_repair((decoder->word2[x] >> 16) & 0x0000ffff);

        if (ec1 != -1 && ec2 != -1 && ec3 != -1 && ec4 != -4) {
            switch (decoder->fs2state[x]) {
            case 0:
                decoder->fs2w1[x] = (((ec1 >> 8) & 0xf) << 28) | (((ec2 >> 8) & 0xf) << 24) |
                                    (((ec3 >> 8) & 0xf) << 20) | (((ec4 >> 8) & 0xf) << 16);
                decoder->fs2state[x] = 1;
                decoder->shstate[x]  = 1;
                decoder->shcount[x]  = 32;
                break;
            case 1:
                decoder->fs2w1[x] |= (((ec1 >> 8) & 0xf) << 12) | (((ec2 >> 8) & 0xf) << 8) |
                                     (((ec3 >> 8) & 0xf) <<  4) | (((ec4 >> 8) & 0xf));
                decoder->fs2state[x] = 2;
                decoder->shstate[x]  = 1;
                decoder->shcount[x]  = 32;
                break;
            case 2:
                decoder->fs2w2[x] = (((ec1 >> 8) & 0xf) << 28) | (((ec2 >> 8) & 0xf) << 24) |
                                    (((ec3 >> 8) & 0xf) << 20) | (((ec4 >> 8) & 0xf) << 16);
                decoder->fs2state[x] = 3;
                decoder->shstate[x]  = 1;
                decoder->shcount[x]  = 32;
                break;
            case 3:
                decoder->fs2w2[x] |= (((ec1 >> 8) & 0xf) << 12) | (((ec2 >> 8) & 0xf) << 8) |
                                     (((ec3 >> 8) & 0xf) <<  4) | (((ec4 >> 8) & 0xf));
                crc = _fsync_crc(decoder->fs2w1[x], decoder->fs2w2[x]);
                if (crc == (int)(decoder->fs2w2[x] & 0x0000ffff)) {
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w1[x] >> 24) & 0xff;
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w1[x] >> 16) & 0xff;
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w1[x] >>  8) & 0xff;
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w1[x] >>  0) & 0xff;
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w2[x] >> 24) & 0xff;
                    decoder->message[x][decoder->msglen[x]++] = (decoder->fs2w2[x] >> 16) & 0xff;
                    decoder->is_fs2[x]   = 1;
                    decoder->fs2state[x] = 0;
                    decoder->shstate[x]  = 1;
                    decoder->shcount[x]  = 32;
                } else {
                    decoder->actives--;
                    if (decoder->msglen[x] > 0 && !decoder->actives)
                        _dispatch(decoder, x);
                    decoder->shstate[x]  = 0;
                    decoder->fs2state[x] = 0;
                }
                break;
            default:
                decoder->fs2state[x] = 0;
                break;
            }
        } else {
            decoder->actives--;
            if (decoder->msglen[x] > 0 && !decoder->actives)
                _dispatch(decoder, x);
            decoder->shstate[x]  = 0;
            decoder->fs2state[x] = 0;
        }
    }
}

static int _onebits(fsync_u32_t n)
{
    int i = 0;
    while (n) { ++i; n &= (n - 1); }
    return i;
}

static void _shiftin(fsync_decoder_t *decoder, int x)
{
    int bit = decoder->xorb[x];

    decoder->synchigh[x] <<= 1;
    if (decoder->synclow[x] & 0x80000000)
        decoder->synchigh[x] |= 1;
    decoder->synclow[x] <<= 1;
    if (bit)
        decoder->synclow[x] |= 1;

    switch (decoder->shstate[x]) {
    case 0:
        if (_onebits(decoder->synchigh[x] ^ 0xaaaa23eb) < FSYNC_GDTHRESH ||
            _onebits(decoder->synchigh[x] ^ 0xaaaa052b) < FSYNC_GDTHRESH) {
            decoder->actives++;
            decoder->word1[x]    = decoder->synclow[x];
            decoder->shstate[x]  = 2;
            decoder->shcount[x]  = 32;
            decoder->msglen[x]   = 0;
            decoder->fs2state[x] = 0;
        }
        return;
    case 1:
        if (--decoder->shcount[x] <= 0) {
            decoder->word1[x]   = decoder->synclow[x];
            decoder->shcount[x] = 32;
            decoder->shstate[x] = 2;
        }
        return;
    case 2:
        if (--decoder->shcount[x] <= 0) {
            decoder->word2[x] = decoder->synclow[x];
            _procbits(decoder, x);
        }
        return;
    default:
        decoder->shstate[x]  = 0;
        decoder->fs2state[x] = 0;
        return;
    }
}

static void _zcproc(fsync_decoder_t *decoder, int x)
{
    if (x >= FSYNC_ND_12) {
        switch (decoder->zc[x]) {
        case 1: decoder->xorb[x] = 1; break;
        case 2: decoder->xorb[x] = 0; break;
        default: return;
        }
    } else {
        switch (decoder->zc[x]) {
        case 2: case 4: decoder->xorb[x] = 1; break;
        case 3:         decoder->xorb[x] = 0; break;
        default: return;
        }
    }
    _shiftin(decoder, x);
}

int fsync_decoder_process_samples(fsync_decoder_t *decoder,
                                   fsync_sample_t  *samples,
                                   int numSamples)
{
    fsync_int_t   i, j, k;
    fsync_sample_t sample;
    fsync_float_t  value, delta;

    if (!decoder) return -1;

    for (i = 0; i < numSamples; i++) {
        sample = samples[i];

#if defined(FSYNC_SAMPLE_FORMAT_U8)
        value = (((fsync_float_t)sample) - 128.0) / 256.0;
#elif defined(FSYNC_SAMPLE_FORMAT_S8)
        value = ((fsync_float_t)sample) / 256.0;
#elif defined(FSYNC_SAMPLE_FORMAT_U16)
        value = (((fsync_float_t)sample) - 32768.0) / 65536.0;
#elif defined(FSYNC_SAMPLE_FORMAT_S16)
        value = ((fsync_float_t)sample) / 65536.0;
#elif defined(FSYNC_SAMPLE_FORMAT_FLOAT)
        value = sample;
#else
#error "no known sample format set"
#endif

#ifdef DIFFERENTIATOR
        delta = value - decoder->lastvalue;
        decoder->lastvalue = value;

        if (decoder->level == 0) {
            if (delta > decoder->hyst) {
                for (k = 0; k < FSYNC_ND; k++) decoder->zc[k]++;
                decoder->level = 1;
            }
        } else {
            if (delta < (-1.0 * decoder->hyst)) {
                for (k = 0; k < FSYNC_ND; k++) decoder->zc[k]++;
                decoder->level = 0;
            }
        }
#else
        if (decoder->level == 0) {
            if (value > decoder->hyst) {
                for (k = 0; k < FSYNC_ND; k++) decoder->zc[k]++;
                decoder->level = 1;
            }
        } else {
            if (value < (-1.0 * decoder->hyst)) {
                for (k = 0; k < FSYNC_ND; k++) decoder->zc[k]++;
                decoder->level = 0;
            }
        }
#endif

        for (j = 0; j < FSYNC_ND; j++) {
            int th_zero, th_one;
            if (j < FSYNC_ND_12) {
                th_zero = (int)((256.0 / TWOPI) * decoder->thx[j]);
                th_one  = (int)((256.0 / TWOPI) * decoder->th[j]);
            } else {
                th_zero = (int)((256.0 / TWOPI) * decoder->th[j]);
                th_one  = (int)((256.0 / TWOPI) * decoder->thx[j]);
            }

            decoder->accum0[j]  += _sintable[th_zero] * value;
            decoder->accum1[j]  += _sintable[th_one]  * value;
            decoder->accum0i[j] += _sintable[(th_zero + 64) % 256] * value;
            decoder->accum1i[j] += _sintable[(th_one  + 64) % 256] * value;

            if (j < FSYNC_ND_12) {
                decoder->th[j]  += decoder->incr;
                decoder->thx[j] += 1.5 * decoder->incr;
            } else {
                decoder->th[j]  += 2.0 * decoder->incr;
                decoder->thx[j] += decoder->incr;
            }

            while (decoder->thx[j] >= TWOPI) decoder->thx[j] -= TWOPI;

            if (decoder->th[j] >= TWOPI) {
                if ((decoder->accum0[j]  * decoder->accum0[j]  +
                     decoder->accum0i[j] * decoder->accum0i[j]) >
                    (decoder->accum1[j]  * decoder->accum1[j]  +
                     decoder->accum1i[j] * decoder->accum1i[j]))
                    decoder->xorb[j] = 0;
                else
                    decoder->xorb[j] = 1;

                decoder->accum0[j] = decoder->accum1[j] = 0.0;
                decoder->accum0i[j] = decoder->accum1i[j] = 0.0;
                _shiftin(decoder, j);
                decoder->th[j] -= TWOPI;
            }
        }
        (void)_zcproc; // suppress unused-function warning
    }

    return 0;
}

int fsync_decoder_end_samples(fsync_decoder_t *decoder)
{
    int i, j;
    if (!decoder) return -1;
    for (i = 0; i < 70; i++)
        for (j = 0; j < FSYNC_ND; j++)
            _shiftin(decoder, j);
    return 0;
}

int fsync_decoder_set_callback(fsync_decoder_t *decoder,
                                fsync_decoder_callback_t callback_function,
                                void *context)
{
    if (!decoder) return -1;
    decoder->callback         = callback_function;
    decoder->callback_context = context;
    return 0;
}

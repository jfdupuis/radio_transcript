/*-
 * fsync_types.h
 *  common typedef header for fsync_decode.h, fsync_encode.h
 *
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2012, 2013, 2014  Matthew Kaufman  All rights reserved.
 *
 * This file is part of Matthew Kaufman's fsync Encoder/Decoder Library.
 * GPL-2.0 — see COPYING for full license text.
 */

#ifndef _FSYNC_TYPES_H_
#define _FSYNC_TYPES_H_

typedef int           fsync_s32;
typedef unsigned int  fsync_u32_t;
typedef short         fsync_s16_t;
typedef unsigned short fsync_u16_t;
typedef char          fsync_s8_t;
typedef unsigned char fsync_u8_t;
typedef double        fsync_float_t;
typedef int           fsync_int_t;

/* Sample format: unsigned 8-bit (matches rtl_fm | sox output) */
typedef unsigned char fsync_sample_t;
#define FSYNC_SAMPLE_FORMAT_U8

#endif /* _FSYNC_TYPES_H_ */

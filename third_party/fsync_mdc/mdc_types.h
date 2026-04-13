/*-
 * mdc_types.h — common typedef header for mdc_decode.h.
 * Author: Matthew Kaufman (matthew@eeph.com)
 * Copyright (c) 2010. GPL-2.0.
 */

#ifndef _MDC_TYPES_H_
#define _MDC_TYPES_H_

typedef int            mdc_s32;
typedef unsigned int   mdc_u32_t;
typedef short          mdc_s16_t;
typedef unsigned short mdc_u16_t;
typedef char           mdc_s8_t;
typedef unsigned char  mdc_u8_t;
typedef int            mdc_int_t;
typedef double         mdc_float_t;

/* Sample format: unsigned 8-bit */
typedef unsigned char mdc_sample_t;
#define MDC_SAMPLE_FORMAT_U8

#endif /* _MDC_TYPES_H_ */

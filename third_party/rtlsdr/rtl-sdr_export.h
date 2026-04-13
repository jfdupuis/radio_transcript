/*
 * Platform export macros for rtl-sdr.
 * Vendored from osmocom/rtl-sdr (GPL-2.0+)
 */
#ifndef __RTL_SDR_EXPORT_H
#define __RTL_SDR_EXPORT_H

#if defined(__GNUC__) || defined(__clang__)
#  define RTLSDR_API __attribute__((visibility("default")))
#elif defined(_MSC_VER)
#  if defined(rtlsdr_EXPORTS)
#    define RTLSDR_API __declspec(dllexport)
#  else
#    define RTLSDR_API __declspec(dllimport)
#  endif
#else
#  define RTLSDR_API
#endif

#endif /* __RTL_SDR_EXPORT_H */

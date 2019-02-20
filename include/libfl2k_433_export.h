/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                           librtl_433                            *
 *                                                                 *
 *    A library to facilitate the use of osmo-fl2k for OOK-based   *
 *    RF transmissions                                             *
 *                                                                 *
 *    coded in 2018/19 by winterrace (github.com/winterrace)       *
 *                                   (github.com/winterrace2)      *
 *                                                                 *
 * This program is free software; you can redistribute it and/or   *
 * modify it under the terms of the GNU General Public License as  *
 * published by the Free Software Foundation; either version 2 of  *
 * the License, or (at your option) any later version.             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef FL2K_433_EXPORT_H
#define FL2K_433_EXPORT_H

#if defined __GNUC__
#  if __GNUC__ >= 4
#    define __FL2K_433_EXPORT   __attribute__((visibility("default")))
#    define __FL2K_433_IMPORT   __attribute__((visibility("default")))
#  else
#    define __FL2K_433_EXPORT
#    define __FL2K_433_IMPORT
#  endif
#elif _MSC_VER
#  define __FL2K_433_EXPORT     __declspec(dllexport)
#  define __FL2K_433_IMPORT     __declspec(dllimport)
#else
#  define __FL2K_433_EXPORT
#  define __FL2K_433_IMPORT
#endif

#ifndef libfl2k_433_STATIC
#	ifdef libfl2k_433_EXPORTS
#	define FL2K_433_API __FL2K_433_EXPORT
#	else
#	define FL2K_433_API __FL2K_433_IMPORT
#	endif
#else
#define FL2K_433_API
#endif
#endif /* FL2K_433_EXPORT_H */

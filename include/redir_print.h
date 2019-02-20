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

#ifndef FL2K_433_REDIR_PRINT_H
#define FL2K_433_REDIR_PRINT_H

#define LOG_TRG_STDERR 1
#define LOG_TRG_STDOUT 2

#include <stdio.h>
#include "libfl2k_433_export.h"

typedef void(*std_print_wrapper)(char target, char *text, void *ctx);

/* Configure a redirection for data printed to stdout or stderr
 * \param cb callback function to receive printed data
 * \param ctx user specific context to pass via the callback function
 * \return 0 on success
 */
FL2K_433_API int fl2k433_print_redirection(std_print_wrapper cb, void *ctx);

/*
 * fprintf wrapper. If there's a registered redirection, it prints to a buffer
 * and sends it to the callback.
 * otherwise normal output to specified stream.
 */
int fl2k433_fprintf(FILE *stream, const char* aFormat, ...);

#endif // FL2K_433_REDIR_PRINT_H

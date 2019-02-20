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

#include <stdarg.h>
#include <malloc.h>
#include "redir_print.h"

static std_print_wrapper print_cb = NULL; // registered callback function. Initially unset.
static void *context = NULL; // registered context. Initially unset.

/*
 * allow an external caller to register a redirection callback
 * to receive stdout or stderr output
 */
FL2K_433_API int fl2k433_print_redirection(std_print_wrapper cb, void *ctx) {
	if (!cb) return -1;
	print_cb = cb;
	context = ctx;
	return 0;
}

/*
 * fprintf wrapper. If there's a registered redirection, it prints to a buffer
 * and sends it to the callback.
 * otherwise normal output to specified stream.
 */
int fl2k433_fprintf(FILE *stream, const char* aFormat, ...) {
	va_list argptr;
	int rv = 0;

	va_start(argptr, aFormat);
	// if a callback is registered, pass stderr/stdout data to it
	if (print_cb && (stream == stdout || stream == stderr)) {
		static char printbuf[512]; // local default buffer (stack). Sufficient size for most use cases.
		char *buf = printbuf; // we use the local buffer if possible
		rv = vsnprintf(NULL, 0, aFormat, argptr) + 1; // test how much space we really need
		int needed_cap = rv;
		if (needed_cap >= 0) {
			int need_more_mem = (needed_cap > sizeof(printbuf) ? 1 : 0); // if we need more space, we...
			if (need_more_mem) buf = calloc(1, needed_cap + 10); // ...allocate our buffer dynamically on the heap
			rv = vsprintf(buf, aFormat, argptr);
			// call the callback function
			if (rv >= 0) print_cb((stream == stderr ? LOG_TRG_STDERR : LOG_TRG_STDOUT), buf, context);
			// free the dynamic buffer (if used)
			if (need_more_mem) free(buf);
		}
	}
	// otherwise write it to the output stream
	else {
		rv = vfprintf(stream, aFormat, argptr);
	}
	va_end(argptr);

	if (rv < 0) rv = -1;
	return rv;
}

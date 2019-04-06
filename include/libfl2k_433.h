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

#ifndef LIBFL2K_433_H
#define LIBFL2K_433_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>

#include "libfl2k_433_export.h"
#include "osmo-fl2k.h"
#include "sinegen.h"
#include "redir_print.h"

#define FL2K_433_DEFAULT_SAMPLE_RATE 85555554
#define FL2K_433_DEFAULT_CARRIER1 6183693
#define FL2K_433_DEFAULT_CARRIER2 0 // 0 = disabled
#define FL2K_433_DEFAULT_DEV_IDX 0
#define FL2K_433_DEFAULT_VERBOSITY 1
#define FL2K_433_DEFAULT_INIT_TIME 200

#define MAX_PATHLEN 300

typedef enum {
	FL2K433_STOPPED = 0,
	FL2K433_STARTUP_FL2K,
	FL2K433_STARTUP_FILE,
	FL2K433_RUNNING_FL2K,
	FL2K433_RUNNING_FILE,
} fl2k433_state;

#define FL2K_433_ERROR_INVALID_PARAM -99
#define FL2K_433_ERROR_INTERNAL -98
#define FL2K_433_ERROR_OUTOFMEM -97

	// fl2k_433 configuration
	typedef struct _fl2k433cfg {
		int dev_index;				// device index of FL2K device to be used
		char out_dir[MAX_PATHLEN];	// target directory for file mode
		uint32_t samp_rate;			// sample rate. Max value depends on the USB(3) chipset (around 150 MHz)
		uint32_t carrier1;			// primary carrier frequency (OOK + FSK)
		uint32_t carrier2;			// secondary carrier frequency (FSK)
		uint8_t verbose;			// debug level. 0 = silent
		uint32_t inittime_ms;		// milliseconds to wait for fl2k to initialize before transmitting actual payload
	} fl2k433cfg, *pfl2k433cfg;

	// Configuration of the FL2K chipset in terms if achievable sample rate
	typedef struct _Fl2kCfg {
		uint32_t sample_clock;
		uint8_t mult;
		uint8_t div;
		uint8_t frac;
	}Fl2kCfg, *pFl2kCfg;

	// Relevant modulation types
	typedef enum {
		MODULATION_TYPE_NONE = 0, // invalid modulation types (for internal use)
		MODULATION_TYPE_OOK  = 1, // Non-null sample: Send primary carrier frequency. Null sample: Send nothing (0 MHz)
		MODULATION_TYPE_FSK  = 2, // Non-null sample: Send primary carrier frequency. Null sample: Send secondary carrier frequency
		MODULATION_TYPE_SINE = 3  // Output a continuous sine wave (for testing purposes)
	} mod_type;

	// Tx messages that can be queued in in the fl2k_433 instance
	typedef struct _TxMsg TxMsg;
	typedef struct _TxMsg {
		mod_type mod;
		char *buf;
		uint32_t len;
		uint32_t samp_rate;
		TxMsg *next;
	}TxMsg, *pTxMsg;

	typedef struct _fl2k_data_info_fm_t { // extended version of fl2k_data_info_t for file mode
		fl2k_data_info_t di;
		mod_type msg_mod;      // != MODULATION_TYPE_NONE if a message is contained
		int      msg_finished; // > 0 if the message was sent completely
	} fl2k_data_info_fm_t;

typedef struct _fl2k_433 {
	// public:
	fl2k433cfg cfg;

	// private:
	fl2k_dev_t *dev;				// Handle to current device
	volatile fl2k433_state opstate;	// signals active operation mode (TX or file mode)
	volatile int cancel_filemode;	// signal to cancel file mode. Not valid in FL2K mode
	unsigned long starttime;		// timestamp set at txstart for checking cfg->inittime_ms. Only valid in FL2K mode (not in file mode)

									/* TX queue */
	TxMsg    *txqueue;				// Queue (linked list) with TX messages that shall be sent (new ones are appended at the end)
	uint32_t  txqueue_sent;			// Number of bytes of current object (first in queue) that have already been sent

									/* TX buffer */
	char txbuf[FL2K_BUF_LEN];		// tx buffer. Filled and passed to libosmo-fl2k by fl2k_callback.

	SineGen *sg;
} fl2k_433_t;

//public methods
FL2K_433_API int			fl2k_433_init(fl2k_433_t **out_fl2k);		// Creates a new fl2k_433 instance
FL2K_433_API int			fl2k_433_destroy(fl2k_433_t *fl2k);			// Frees the instance
FL2K_433_API int			txstart(fl2k_433_t *fl2k);					// Starts transmission mode. Blocks until finished or got stopped
FL2K_433_API int			txstop_signal(fl2k_433_t *fl2k);			// Signals a stop request
FL2K_433_API int			QueueTxMsg(fl2k_433_t *fl2k, TxMsg *msg);	// Queues a message to be TXed
FL2K_433_API int			getQueueLength(fl2k_433_t *fl2k);
FL2K_433_API fl2k433_state	getState(fl2k_433_t *fl2k);

// non-member (instance-independent) functions:
FL2K_433_API void	getCfgTables(pFl2kCfg *useable, uint32_t *n_useable, pFl2kCfg *redundant, uint32_t *n_redundant);

#ifdef __cplusplus
}
#endif

#endif /* LIBFL2K_433_H */

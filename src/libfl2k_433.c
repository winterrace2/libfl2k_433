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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef _WIN32
#include <unistd.h>
#define sleep_ms(ms)	usleep(ms*1000)
#else
#include <windows.h>
#include <io.h>
#define sleep_ms(ms)	Sleep(ms)
#ifdef _MSC_VER
#define F_OK 0
#endif
#endif

#include "libfl2k_433.h"
#include "redir_print.h"

#define FILEMODE_SLEEP_TIME 50

// forward declaration of private methods (not in header)
static void		fl2k_callback(fl2k_data_info_t *data_info);	// Callback function for libosmo-fl2k
static int		InitFl2k(fl2k_433_t *fl2k);				// Initializes the FL2K device using libosmo-fl2k
static void		loadDefaultConfig(fl2k_433_t *fl2k);	// Loads the default configuration
static int		InitCarrier(fl2k_433_t *fl2k);
static TxMsg*	TxPop(fl2k_433_t *fl2k);
static void		TxPush(fl2k_433_t *fl2k, TxMsg *msg);
static int isBufFilled(char *buf, int len);
static FILE *openOutputFile(char *dir, uint32_t samp_rate, uint32_t carrier, uint32_t filenum);
static void *file_mode(fl2k_433_t *fl2k);

FL2K_433_API int	fl2k_433_init(fl2k_433_t **out_fl2k) {
	if (!out_fl2k) {
		fl2k433_fprintf(stderr, "fl2k_433_init: mandatory parameter is not set.\n");
		return FL2K_433_ERROR_INVALID_PARAM;
	}

	fl2k_433_t *fl2k = (fl2k_433_t*)calloc(1, sizeof(fl2k_433_t));
	if (fl2k) {
		fl2k->opstate = FL2K433_STOPPED;
		loadDefaultConfig(fl2k);
	}
	*out_fl2k = fl2k;
	//todo: print version?
	return 0;
}

FL2K_433_API int fl2k_433_destroy(fl2k_433_t *fl2k) {
	// check object
	if (!fl2k) {
		fl2k433_fprintf(stderr, "fl2k_433_destroy: missing context.\n");
		return FL2K_433_ERROR_INVALID_PARAM;
	}

	// free queue
	TxMsg *m = TxPop(fl2k);
	while (m != NULL) {
		if (m->buf) free(m->buf);
		free(m);
		m = TxPop(fl2k);
	}

	// free carrier
	if (fl2k->carrier_buf) {
		free(fl2k->carrier_buf);
		fl2k->carrier_buf = NULL;
	}

	// free object
	free(fl2k);
	return 0;
}

FL2K_433_API int getState(fl2k_433_t *fl2k) {
	return fl2k->opstate;
}

static void loadDefaultConfig(fl2k_433_t *fl2k) {
	fl2k->cfg.dev_index = FL2K_433_DEFAULT_DEV_IDX;
	fl2k->cfg.samp_rate = FL2K_433_DEFAULT_SAMPLE_RATE;
	fl2k->cfg.carrier = FL2K_433_DEFAULT_CARRIER;
	fl2k->cfg.verbose = FL2K_433_DEFAULT_VERBOSITY;
	memset(fl2k->cfg.out_dir, 0, sizeof(fl2k->cfg.out_dir));
	fl2k->cfg.inittime_ms = FL2K_433_DEFAULT_INIT_TIME;
}

static int InitCarrier(fl2k_433_t *fl2k) {
	// Sanity / context checks
	if (fl2k->cfg.carrier < CARRIER_MIN) {
		fl2k433_fprintf(stderr, "InitCarrier: Carrier frequency is too low.\n");
		return 0;
	}
	if (fl2k->carrier_buf && fl2k->carrier_freq == fl2k->cfg.carrier) {
		if (fl2k->cfg.verbose > 1) fl2k433_fprintf(stderr, "Carrier from last run can be reused.\n");
		return 1;// carrier is already set, we can simple (re)use it.
	}

	// Calculate required number of samples for a whole sine wave length and check against Nyquist
	double samplesPerCycle = (double)fl2k->cfg.samp_rate / (double)fl2k->cfg.carrier;
	if (samplesPerCycle < 2.0 && fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "Warning: Carrier frequency %lu higher than %lu, violating Nyquist theoreom.\n", fl2k->cfg.carrier, (fl2k->cfg.samp_rate + 1) / 2);

	// Find a sine wave sequence (in the desired length interval) which ends between two sample points (so the signal buffer can be repeated with little unwanted frequencies)
	int cycles_min = ((int)((double)FL2K_BUF_LEN / samplesPerCycle)) + 1;
	int cycles_max = ((int)((double)MAX_CARRIER_ALLOC / samplesPerCycle)) - 1;
	int cycles_opt = 0;
	double diff_opt = 0.5;
	for (int c = cycles_min; c < cycles_max; c++) {
		double samples = samplesPerCycle * (double)c;
		double mod = fmod(samples, 1.0);
		double diff = min(mod, 1.0 - mod);
		if (diff < diff_opt) {
			diff_opt = diff;
			cycles_opt = c;
		}
	}
	if (fl2k->cfg.verbose > 1) fl2k433_fprintf(stderr, "Found optimal carrier with %lu cycles (min diff = %f).\n", cycles_opt, diff_opt);

	// Initialize / allocate and fill the carrier buffer
	fl2k->carrier_len = (int)(0.5 + ((double)cycles_opt * samplesPerCycle));
	fl2k->carrier_freq = fl2k->cfg.carrier;
	fl2k->carrier_pos = 0;
	fl2k->carrier_buf = (char*)malloc(fl2k->carrier_len);
	if (!fl2k->carrier_buf) {
		fl2k433_fprintf(stderr, "Memory for carrier buffer could not be allocated.\n");
		return 0;
	}
	for (uint32_t s = 0; s < fl2k->carrier_len; s++) {
		double current_radian = M_PI * s * 2 / samplesPerCycle;
		double carrier_sin_value = sin(current_radian);
		fl2k->carrier_buf[s] = (int8_t)(carrier_sin_value * 127.0);
	}
	return 1;
}

static TxMsg *TxPop(fl2k_433_t *fl2k) {
	TxMsg *msg = fl2k->txqueue;
	if (msg) {
		fl2k->txqueue = msg->next;
		fl2k->txqueue_sent = 0;
		msg->next = NULL;
	}
	return msg;
}

static void TxPush(fl2k_433_t *fl2k, TxMsg *msg) {
	TxMsg **ptr = &fl2k->txqueue;
	while (*ptr) ptr = &(*ptr)->next;
	*ptr = msg;
}

// important: target sample rate must have already been set when queuing a TX message
FL2K_433_API int QueueTxMsg(fl2k_433_t *fl2k, TxMsg *msg_in) {
	int r = -1;
	if (!msg_in || (msg_in->mod != MODULATION_TYPE_SINE && (!msg_in->buf || msg_in->len < 1 || msg_in->next))) {
		fl2k433_fprintf(stderr, "QueueTxMsg: Malformed TX message object can not be queued\n");
		return r;
	}

	if (msg_in->mod == MODULATION_TYPE_SINE) {
		TxMsg *msg_out = calloc(1, sizeof(TxMsg));
		msg_out->mod = msg_in->mod;
		TxPush(fl2k, msg_out);
		r = 0;
	}
	else if (msg_in->mod == MODULATION_TYPE_OOK) {
		TxMsg *msg_out = calloc(1, sizeof(TxMsg));
		msg_out->mod = msg_in->mod;
		msg_out->samp_rate = fl2k->cfg.samp_rate;
		double scale_factor = (double)msg_out->samp_rate / (double)msg_in->samp_rate;
		msg_out->len = (int)((double)msg_in->len * scale_factor);
		msg_out->buf = (char*)malloc(msg_out->len);
		uint32_t trgidx1 = 0; // will carry a * scale_factor
		for (uint32_t a = 0; a < (msg_in->len - 1); a++) {
			uint32_t trgidx2 = (int)((double)(a + 1) * scale_factor);
			uint32_t trgidxm = trgidx1 + ((trgidx2 - trgidx1) / 2);
			uint32_t t1safe = min(trgidx1, msg_out->len - 1);
			uint32_t t2safe = min(trgidx2, msg_out->len - 1);
			uint32_t tmsafe = min(trgidxm, msg_out->len - 1);
			for (uint32_t b = t1safe; b < tmsafe; b++) {
				msg_out->buf[b] = msg_in->buf[a];
			}
			for (uint32_t b = tmsafe; b < t2safe; b++) {
				msg_out->buf[b] = msg_in->buf[a + 1];
			}
			trgidx1 = trgidx2;
		}
		for (uint32_t a = trgidx1; a < msg_out->len; a++) {
			msg_out->buf[a] = msg_in->buf[msg_in->len - 1];
		}
		TxPush(fl2k, msg_out);
		r = 0;
	}
	else if (msg_in->mod == MODULATION_TYPE_OOK) {
		fl2k433_fprintf(stderr, "QueueTxMsg: FSK modulation is not supported, yet.\n");
	}
	else {
		fl2k433_fprintf(stderr, "QueueTxMsg: TX message can not be queued due to unknown modulation type\n");
	}
	return r;
}

FL2K_433_API int getQueueLength(fl2k_433_t *fl2k) {
	int num = 0;
	TxMsg **ptr = &fl2k->txqueue;
	while (*ptr) {
		num++;
		ptr = &(*ptr)->next;
	}
	return num;
}

static unsigned long getMilliSeconds() {
#ifdef _WIN32
	return GetTickCount();
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (1000 * tv.tv_sec) + (tv.tv_usec / 1000);
#endif
}

static char zero_buf[FL2K_BUF_LEN] = { 0 }; // empty buffer as fallback (errors like missing context, ...) or if no more payload is waiting to be sent

static void fl2k_callback(fl2k_data_info_t *data_info) {
	if (!data_info || !data_info->ctx) return;

	data_info->sampletype_signed = 1;
	data_info->r_buf = zero_buf; // worst cases > good cases, so we choose the zero array by default

	// check context and fill data_info
	fl2k_433_t *fl2k = (fl2k_433_t*)data_info->ctx;
	if(!fl2k){
		fl2k433_fprintf(stderr, "fl2k_callback: Missing context, providing NULL samples.\n");
		return;
	}
	if (!fl2k->carrier_buf) {
		fl2k433_fprintf(stderr, "fl2k_callback: Missing carrier buffer, providing NULL samples.\n");
		return;
	}

	// if we are in device mode, give the adapter some time to initialize (output nullsamples only)
	if (fl2k->cfg.inittime_ms > 0 && fl2k->opstate == FL2K433_STARTUP_FL2K && fl2k->starttime && getMilliSeconds() < (fl2k->starttime + fl2k->cfg.inittime_ms)) {
		return;
	}
	if (fl2k->opstate == FL2K433_STARTUP_FL2K && fl2k->starttime) {
		fl2k->starttime = 0;
	}

	// startup state ends here
	if      (fl2k->opstate == FL2K433_STARTUP_FL2K) fl2k->opstate = FL2K433_RUNNING_FL2K;
	else if (fl2k->opstate == FL2K433_STARTUP_FILE) fl2k->opstate = FL2K433_RUNNING_FILE;

	// 	If there's nothing to do: output nullsamples only
	if (!fl2k->txqueue) return;

	// theres a TX message in queue. Some basic checks, then prepare the signal...
	if (fl2k->txqueue->mod < MODULATION_TYPE_SINE || fl2k->txqueue->mod > MODULATION_TYPE_FSK) {
		fl2k433_fprintf(stderr, "fl2k_callback: Unknown modulation type.\n");
		return;
	}
	if (fl2k->txqueue->mod != MODULATION_TYPE_SINE && (!fl2k->txqueue->buf || !fl2k->txqueue->len)) {
		fl2k433_fprintf(stderr, "fl2k_callback: Unexpected condition, TX message has no data.\n");
		return;
	}

	// 1a) Fill signal with sine (OOK & test sine):
	if (fl2k->txqueue->mod == MODULATION_TYPE_OOK || fl2k->txqueue->mod == MODULATION_TYPE_SINE) {
		data_info->r_buf = fl2k->txbuf;

		// Copy carrier segment to txbuf
		uint32_t part1len = min(fl2k->carrier_len - fl2k->carrier_pos, FL2K_BUF_LEN);
		memcpy(fl2k->txbuf, &fl2k->carrier_buf[fl2k->carrier_pos], part1len);
		fl2k->carrier_pos = (fl2k->carrier_pos + part1len) % fl2k->carrier_len;
		if (part1len < FL2K_BUF_LEN) {
			uint32_t part2len = min(fl2k->carrier_len - fl2k->carrier_pos, (FL2K_BUF_LEN - part1len));
			memcpy(&fl2k->txbuf[part1len], &fl2k->carrier_buf[fl2k->carrier_pos], part2len);
			fl2k->carrier_pos = (fl2k->carrier_pos + part2len) % fl2k->carrier_len;
			if ((part1len + part2len) > FL2K_BUF_LEN) {
				fl2k433_fprintf(stderr, "fl2k_callback: Unexpected condition, carrier buffer too small.\n");
			}
		}
		// don't repeat sine wave forever in filemode
		if (fl2k->txqueue->mod == MODULATION_TYPE_SINE) {
			if (fl2k->opstate == FL2K433_RUNNING_FILE) {
				TxMsg *m = TxPop(fl2k);
				free(m);
			}
		}
		// 1b) Mask with TX samples (OOK)
		else if (fl2k->txqueue->mod == MODULATION_TYPE_OOK) {
			if(fl2k->cfg.verbose > 1 && fl2k->txqueue_sent == 0) fl2k433_fprintf(stderr, "fl2k_callback: start sending an OOK signal.\n");
			// Mask with TX samples
			char *mask = &fl2k->txqueue->buf[fl2k->txqueue_sent];
			char *end = &fl2k->txqueue->buf[fl2k->txqueue->len];
			for (uint32_t a = 0; a < sizeof(fl2k->txbuf) /*FL2K_BUF_LEN*/; a++) {
				if (&mask[a] >= end || mask[a] == 0) fl2k->txbuf[a] = 0;
			}
			fl2k->txqueue_sent += sizeof(fl2k->txbuf);
			// If we are ready with the TX message, remove it and free its memory
			if (fl2k->txqueue_sent >= fl2k->txqueue->len) {
				if(fl2k->cfg.verbose > 1) fl2k433_fprintf(stderr, "fl2k_callback: finished sending an OOK signal.\n");
				TxMsg *m = TxPop(fl2k); // will clear txqueue_sent
				if(m->buf) free(m->buf);
				free(m);
			}
		}
	}
	// 2) FSK
	else if (fl2k->txqueue->mod == MODULATION_TYPE_FSK) {
		if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "fl2k_callback: FSK modulation is not supported, yet.\n");
		// remove FSK message
		TxMsg *m = TxPop(fl2k);
		if (m->buf) free(m->buf);
		free(m);
	}
	return;
}

FL2K_433_API int txstart(fl2k_433_t *fl2k) {
	int r = 0; // 0 = failure, 1 = success

	if (fl2k->opstate > FL2K433_STOPPED) {
		fl2k433_fprintf(stderr, "start(): fl2k_433 is already running.\n");
		return r;
	}

	if (fl2k->dev) {
		fl2k433_fprintf(stderr, "start(): Unexpected start condition of fl2k_433 object.\n");
		return r;
	}

	// Initialize carrier
	if (!InitCarrier(fl2k)) {
		fl2k433_fprintf(stderr, "start(): Could not initialize carrier.\n");
		return r;
	}

	fl2k->opstate = (fl2k->cfg.out_dir[0] ? FL2K433_STARTUP_FILE : FL2K433_STARTUP_FL2K);
	fl2k->txqueue_sent = 0;

	if(fl2k->opstate == FL2K433_STARTUP_FL2K){
		fl2k->starttime = getMilliSeconds();
		if (InitFl2k(fl2k)) {
			if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "start(): fl2k_433 was started in FL2K mode.\n");
			while (fl2k->opstate != FL2K433_STOPPED) sleep_ms(200);
			r = 1;
		}
		else {
			fl2k433_fprintf(stderr, "start(): FL2K device could not be initialized.\n");
		}
	}
	// Operation (fl2k thread blocks until we're finished)
	else if (fl2k->opstate == FL2K433_STARTUP_FILE) {
		fl2k->cancel_filemode = 0;
		if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "start(): fl2k_433 was started in file mode.\n");
		file_mode(fl2k);
		r = 1;
	}
	fl2k->opstate = FL2K433_STOPPED;
	return r;
}

static int InitFl2k(fl2k_433_t *fl2k) {
	// Select fl2k device by number
	if (fl2k->cfg.dev_index <= 0) {
		fl2k433_fprintf(stderr, "InitFl2k: No FL2K device configured.\n");
		return 0;
	}
	uint16_t device_count = fl2k_get_device_count();
	if (!device_count) {
		fl2k433_fprintf(stderr, "InitFl2k: No supported FL2K devices found.\n");
		return 0;
	}
	else if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "Found %d FL2K device(s)\n", device_count);

	/* Open FL2K device */
	const char *product = NULL;
	if (fl2k->cfg.verbose > 0) {
		product = fl2k_get_device_name(fl2k->cfg.dev_index);
		fl2k433_fprintf(stderr, "trying device  %d:  %s", fl2k->cfg.dev_index, (product ? product : "n/a"));
	}

	if (fl2k_open(&fl2k->dev, fl2k->cfg.dev_index - 1) < 0 || !fl2k->dev) {
		fl2k433_fprintf(stderr, "InitFl2k: Failed to open fl2k device #%d.\n", fl2k->cfg.dev_index);
		return 0;
	}

	if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "Using device %d: %s\n", fl2k->cfg.dev_index, (product ? product : "n/a"));

	/* Start TX thread */
	if (fl2k_start_tx(fl2k->dev, fl2k_callback, fl2k, 0) < 0) {
		fl2k433_fprintf(stderr, "InitFl2k: Failed to start TX thread.\n");
		return 0;
	}

	/* Set the sample rate */
	if (fl2k_set_sample_rate(fl2k->dev, fl2k->cfg.samp_rate) < 0) { // ggf.vor dem Start setzen?
		fl2k433_fprintf(stderr, "InitFl2k: Failed to set sample rate.\n");
		return 0;
	}

	return 1;
}

static int isBufFilled(char *buf, int len) {
	if (!buf) return 0;
	for (int a = 0; a < len; a++) {
		if (buf[a] != 0) return 1;
	}
	return 0;
}

static FILE *openOutputFile(char *dir, uint32_t samp_rate, uint32_t carrier, uint32_t filenum) {
	FILE *r = 0;

	if (!dir) return r;

	// prepare path (append trailing slash if missing)
	char path[300];
	strcpy_s(path, sizeof(path), dir);
	char last = path[strlen(path) - 1];
	if (last != '/' && last != '\\') strcat_s(path, sizeof(path), (strchr(path, '\\') ? "\\" : "/"));

	// add filename
	char *fname = &path[strlen(path)];
	size_t fname_cap = sizeof(path) - strlen(path);
	sprintf_s(fname, fname_cap, "s_%lu_c_%lu_%lu.bin", samp_rate, carrier, filenum); // todo: add time etc.?

	if (_access(path, F_OK) != 0) {
		r = fopen(path, "wb");
		if (!r) fl2k433_fprintf(stderr, "Failed to open %s\n", path);
	}
	else fl2k433_fprintf(stderr, "Output file %s already exists, skipping\n", path);

	return r;
}

void *file_mode(fl2k_433_t *fl2k) {
	fl2k_data_info_t dat_inf;
	dat_inf.ctx = fl2k;
	dat_inf.underflow_cnt = 0;
	dat_inf.len = FL2K_BUF_LEN;
	dat_inf.using_zerocopy = 0;
	dat_inf.device_error = 0;

	FILE *crnt_file = NULL;
	uint32_t num_files = 0;
	while (!fl2k->cancel_filemode) {
		// acquire data
		fl2k_callback(&dat_inf);
		// signal contained?
		if (isBufFilled(dat_inf.r_buf, dat_inf.len)) {
			// Create new file, if necessary
			if (!crnt_file) {
				num_files++;
				crnt_file = openOutputFile(fl2k->cfg.out_dir, fl2k->cfg.samp_rate, fl2k->cfg.carrier, num_files);
			}
			if (crnt_file) {
				// write data
				if (fwrite(dat_inf.r_buf, 1, dat_inf.len, crnt_file) != dat_inf.len) {
					fl2k433_fprintf(stderr, "Short write, samples lost.\n");
				}
			}
		}
		// no signal contained. Close last file (if any)
		else {
			if (crnt_file) {
				fclose(crnt_file);
				crnt_file = NULL;
			}
		}
		sleep_ms(FILEMODE_SLEEP_TIME);
	}
	if (crnt_file) {
		// Close last file
		fclose(crnt_file);
		crnt_file = NULL;
	}
	return NULL;
}

FL2K_433_API int txstop_signal(fl2k_433_t *fl2k) {
	int r = 0;
	if (fl2k->opstate == FL2K433_STOPPED) {
		fl2k433_fprintf(stderr, "stop_signal(): Nothing to stop, fl2k_433 is not running.\n");
	}
	else if (fl2k->opstate == FL2K433_STARTUP_FILE || fl2k->opstate == FL2K433_STARTUP_FL2K) {
		fl2k433_fprintf(stderr, "stop_signal(): Wait until fl2k_433 is fully initialized before trying to stop it.\n");
	}
	else if(fl2k->opstate == FL2K433_RUNNING_FILE){
		fl2k->cancel_filemode = 1;
		for (int a = 0; fl2k->opstate != FL2K433_STOPPED && a < 50; a++) {
			sleep_ms(100); // wait up to 5 seconds
		}
		if (fl2k->opstate == FL2K433_STOPPED) {
			r = 1;
			if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "stop_signal(): file mode thread was stopped.\n");
		}
		else {
			fl2k433_fprintf(stderr, "stop_signal(): file mode thread could not be stopped.\n");
			return r;
		}
	}
	else if (fl2k->opstate == FL2K433_RUNNING_FL2K) {
		int tmp = fl2k_stop_tx(fl2k->dev);
		if (tmp == 0) {
			r = 1;
			if (fl2k->cfg.verbose > 0) fl2k433_fprintf(stderr, "stop_signal(): FL2K TX thread was stopped.\n");
			fl2k->opstate = FL2K433_STOPPED; // this tells the start() thread it may return now;
		}
		else {
			fl2k433_fprintf(stderr, "stop_signal(): FL2K TX thread could not be stopped.\n");
			return r;
		}
	}

	// Clean up
	if (fl2k->dev) {
		fl2k_close(fl2k->dev);
		fl2k->dev = NULL;
	}
	while (fl2k->txqueue) {
		TxMsg *m = TxPop(fl2k);
		if (m->buf) free(m->buf);
		free(m);
	}
	return 1;
}

#define MAX_FL2K_CONFIGS 3725 // required place for 3411 useful and 309 redundant entries
static Fl2kCfg configs[MAX_FL2K_CONFIGS];
static uint32_t n_cfg_useful = 0;
static uint32_t n_cfg_rest = 0;

// this is an unexported routine directly taken from the osmo-fl2k library
static double fl2k_reg_to_freq(uint32_t reg) {
	double sample_clock, offset, offs_div;
	uint32_t pll_clock = 160000000;
	uint8_t div = reg & 0x3f;
	uint8_t out_div = (reg >> 8) & 0xf;
	uint8_t frac = (reg >> 16) & 0xf;
	uint8_t mult = (reg >> 20) & 0xf;

	sample_clock = (pll_clock * mult) / (uint32_t)div;
	offs_div = (pll_clock / 5.0f) * mult;
	offset = ((double)sample_clock / (offs_div / 2)) * 1000000.0f;
	sample_clock += (uint32_t)offset * frac;
	sample_clock /= out_div;

	// fl2k433_fprintf(stderr, "div: %d\tod: %d\tfrac: %d\tmult %d\tclock: %f\treg "
	//			"%08x\n", div, out_div, frac, mult, sample_clock, reg);

	return sample_clock;
}

FL2K_433_API void getCfgTables(pFl2kCfg *useable, uint32_t *n_useable, pFl2kCfg *redundant, uint32_t *n_redundant) {
	// 1) The array of configs needs to be created on first request only
	if (n_cfg_useful == 0) {
		// 1a) Store all configs with their resulting sample rates in the array
		uint8_t out_div = 1; // Comment from osmo-fl2k: Output divider (accepts value 1-15) works, but adds lots of phase noise, so do not use it
		for (uint8_t mult = 6; mult >= 3 && n_cfg_useful < MAX_FL2K_CONFIGS; mult--) { // Comment from osmo-fl2k: Observation: PLL multiplier of 7 works, but has more phase noise. Prefer multiplier 6 and 5
			for (uint8_t div = 63; div > 1 && n_cfg_useful < MAX_FL2K_CONFIGS; div--) {
				for (uint8_t frac = 1; frac <= 15 && n_cfg_useful < MAX_FL2K_CONFIGS; frac++) {
					uint32_t reg = (mult << 20) | (frac << 16) | (0x60 << 8) | (out_div << 8) | div;
					double sample_clock = fl2k_reg_to_freq(reg);
					configs[n_cfg_useful].sample_clock = (uint32_t)sample_clock; // all sample clocks will be .0, so its safe to cast them to an int
					configs[n_cfg_useful].mult = mult;
					configs[n_cfg_useful].div = div;
					configs[n_cfg_useful].frac = frac;
					n_cfg_useful++;
				}
			}
		}
		n_cfg_rest = 0;

		// 1b) Sort the array. Entries of duplicate sample rates will be shifted towards the end (osmo-fl2k will only use/choose 1 setting per sample rate)
		for (uint32_t t = 0; n_cfg_useful > 0 && t < (n_cfg_useful - 1); t++) {
			// For each position: Determine the smallest entry from here till the end of the list
			int smin = t; // assume, the current element is the smallest
			for (uint32_t s = t + 1; s < n_cfg_useful; s++) {
				if (configs[s].sample_clock > configs[smin].sample_clock) continue;
				if (configs[s].sample_clock == configs[smin].sample_clock && (configs[s].mult < configs[smin].sample_clock || configs[s].div < configs[smin].div || configs[s].frac > configs[smin].frac)) continue;
				smin = s; // correct this assumption if we find a smaller one among the rest
			}
			// If the smallest entry was not already in this place, swap both elements
			if (smin != t) {
				Fl2kCfg tmp = configs[t];
				configs[t] = configs[smin];
				configs[smin] = tmp;
			}
			// If the current element yields the same sample rate than its predecessor, move it after the end of the "useful" area
			if (t > 0 && configs[t].sample_clock == configs[t - 1].sample_clock) {
				Fl2kCfg tmp = configs[t];
				configs[t] = configs[n_cfg_useful - 1]; // Replace the current element by the last element of the "useful" area
				configs[n_cfg_useful - 1] = tmp;
				n_cfg_useful--; // Reduce the length of the "useful" area
				n_cfg_rest++; // Increase the lenth of the "rest" area
				t--; // Ensure the next round will work on the same index because the moved element still needs to be sorted
			}
		}
	}
	*useable = &configs[0];
	*n_useable = n_cfg_useful;
	*redundant = &configs[n_cfg_useful];
	*n_redundant = n_cfg_rest;
	return;
}

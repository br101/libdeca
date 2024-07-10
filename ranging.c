/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <math.h>
#include <stdlib.h>

#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwproto.h"
#include "dwtime.h"
#include "dwutil.h"
#include "log.h"
#include "mac802154.h"
#include "ranging.h"

static const char* LOG_TAG = "TWR";

#define TWR_DEBUG_CALCULATION 0
#define TWR_MAX_RETRY		  3
#define TWR_RETRY_DELAY		  20 /* random with this maximum in ms */
#define TWR_SEND_REPORT		  1
#define TWR_SPI_US_PER_BYTE	  2.3 /* TODO: measured with 8MHz DMA for 12 byte */

/*
 * TWR Message definitions
 */
#define TWR_MSG_INIT   0x20
#define TWR_MSG_POLL   0x21
#define TWR_MSG_RESP   0x22
#define TWR_MSG_SSPOLL 0x23
#define TWR_MSG_SSRESP 0x24
#define TWR_MSG_FINA   0x29
#define TWR_MSG_REPO   0x2A

struct twr_msg_final {
	uint32_t round;
	uint32_t delay;
	uint16_t cnum; // sequence number / TWR ID
} __attribute__((packed));

struct twr_msg_ss_resp {
	uint32_t poll_rx_ts;
	uint32_t resp_tx_ts;
} __attribute__((packed));

struct twr_msg_report {
	uint16_t cnum; // sequence number / TWR ID
	uint16_t dist;
} __attribute__((packed));

/* calculated in init */
static uint64_t twr_delay_dtu;
static uint32_t twr_rx_delay; // UUS
static uint16_t twr_pto;

/* state */
static twr_cb_t twr_observer_cb;
static uint64_t twr_dst;
static uint16_t twr_cnum = 0;
static bool single_sided = false;
static uint8_t expected_msg = 0;
static int retry = 0;
static bool in_progress = false;
static uint64_t last_poll_rx_ts;

static void twr_retry(void);
static void twr_handle_timeout(uint32_t status);
static void twr_handle_result(uint64_t src, uint64_t dst, uint16_t dist,
							  uint16_t cnum, bool reported, bool initiator);

static uint64_t twr_my_mac(uint64_t other_addr)
{
	if (IS_SHORT_ADDR(other_addr)) {
		return dwmac_get_mac16();
	} else {
		return dwmac_get_mac64();
	}
}

/*
 * TWR messages
 */

/* TAG -> ANCOR */
static bool twr_send_poll(uint64_t ancor)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	dwprot_prepare(tx, 0, single_sided ? TWR_MSG_SSPOLL : TWR_MSG_POLL, ancor);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_timeout_handler(tx, twr_handle_timeout);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Poll to " LADDR_FMT, LADDR_PAR(ancor));
		expected_msg = single_sided ? TWR_MSG_SSRESP : TWR_MSG_RESP;
	} else {
		LOG_ERR("Failed to send Poll to " LADDR_FMT, LADDR_PAR(ancor));
		twr_retry();
	}

	return res;
}

/* ANCOR -> TAG */
static bool twr_send_response(uint64_t tag, uint64_t poll_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	last_poll_rx_ts = poll_rx_ts;
	uint64_t resp_tx_time = poll_rx_ts + twr_delay_dtu;

	dwprot_prepare(tx, 0, TWR_MSG_RESP, tag);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_txtime(tx, resp_tx_time);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Response to " LADDR_FMT " after %dus", LADDR_PAR(tag),
				(int)DTU_TO_US(resp_tx_time - poll_rx_ts));
		expected_msg = TWR_MSG_FINA;
	} else {
		LOG_ERR("Failed to send Response to " LADDR_FMT, LADDR_PAR(tag));
		LOG_INF_TS("rx_ts", poll_rx_ts);
		LOG_INF_TS("tx_ts", resp_tx_time);
		LOG_INF_TS("delay", twr_delay_dtu);
		expected_msg = 0;
	}

	return res;
}

/* ANCOR -> TAG */
static bool twr_send_ss_response(uint64_t tag, uint64_t poll_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	/* Delayed TX time has a 8ns resolution because the last 9 bit of the
	 * DTU are ignored when programming the delayed TX time. We need to do
	 * the same in the calculated TX time */
	uint64_t resp_tx_time = (poll_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;

	struct twr_msg_ss_resp* msg = dwprot_prepare(
		tx, sizeof(struct twr_msg_ss_resp), TWR_MSG_SSRESP, tag);
	msg->poll_rx_ts = (uint32_t)poll_rx_ts;
	msg->resp_tx_ts = (uint32_t)(resp_tx_time + DWPHY_ANTENNA_DELAY);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_set_txtime(tx, resp_tx_time);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent SS Response to " LADDR_FMT " after %dus", LADDR_PAR(tag),
				(int)DTU_TO_US(resp_tx_time - poll_rx_ts));
		// LOG_DBG_TS("\tPoll RX TS:\t", poll_rx_ts);
		// LOG_DBG_TS("\tResp TX TS:\t", resp_tx_time);
		expected_msg = 0;
	} else {
		LOG_ERR("Failed to send Response to " LADDR_FMT, LADDR_PAR(tag));
		expected_msg = 0;
	}

	return res;
}

/* TAG -> ANCOR */
static bool twr_send_final(uint64_t ancor, uint64_t resp_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	uint64_t poll_tx_ts = dw_get_tx_timestamp();

	/* Delayed TX time has a 8ns resolution because the last 9 bit of the
	 * DTU are ignored when programming the delayed TX time. We need to do
	 * the same in the calculated TX time */
	uint64_t final_tx_time = (resp_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;

	/* Final TX timestamp is the transmission time we programmed plus the TX
	 * antenna delay. */
	uint64_t final_tx_ts = (final_tx_time + DWPHY_ANTENNA_DELAY) & DTU_MASK;

	struct twr_msg_final* final_msg
		= dwprot_prepare(tx, sizeof(struct twr_msg_final), TWR_MSG_FINA, ancor);
	final_msg->cnum = twr_cnum;
	final_msg->round = resp_rx_ts - poll_tx_ts;
	final_msg->delay = final_tx_ts - resp_rx_ts;

	dwmac_tx_set_ranging(tx);
	dwmac_tx_set_txtime(tx, final_tx_time);
#if TWR_SEND_REPORT
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_timeout_handler(tx, twr_handle_timeout);
#endif

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Final to " LADDR_FMT " after %dus", LADDR_PAR(ancor),
				(int)DTU_TO_US(final_tx_time - resp_rx_ts));
		// LOG_DBG_TS("\tPoll TX TS:\t", poll_tx_ts);
		// LOG_DBG_TS("\tResp RX TS:\t", resp_rx_ts);
		// LOG_DBG_TS("\tFina TX TS:\t", final_tx_time);
		expected_msg = TWR_SEND_REPORT ? TWR_MSG_REPO : 0;
	} else {
		LOG_ERR("Failed to send Final");
		twr_retry();
	}

#if !TWR_SEND_REPORT
	/* if reports are not sent by the other side, we assume everything is OK
	 * if the final message was sent. We don't know the distance, so we
	 * just record "OK" */
	twr_handle_result(twr_my_mac(ancor), ancor, TWR_OK_VALUE, twr_cnum, false,
					  true);
	in_progress = false;
#endif

	return res;
}

/* ANCOR -> TAG */
static bool twr_send_report(uint64_t tag, uint16_t dist, uint16_t cnum,
							uint64_t final_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	expected_msg = 0;

	struct twr_msg_report* msg
		= dwprot_prepare(tx, sizeof(struct twr_msg_report), TWR_MSG_REPO, tag);
	msg->cnum = cnum;
	msg->dist = dist;

	uint64_t rep_tx_time = (final_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;
	dwmac_tx_set_txtime(tx, rep_tx_time);

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "Report to " LADDR_FMT ": distance %u cm", LADDR_PAR(tag),
			   dist);

	// we have been the destination of this TWR sequence
	twr_handle_result(tag, twr_my_mac(tag), dist, cnum, false, false);

	return res;
}

/*
 * Distance calculation
 */

static uint16_t twr_fixup_distance(int dist)
{
	if (dist <= -100 || dist > TWR_FAILED_VALUE) { // out of range: -1m - 655m
		return TWR_FAILED_VALUE;
	} else if (dist < 0) {
		/* distance will be unsigned 16 bit from now on, but we allow
		 * for a negative result up to 1m to be "equal" to 0 */
		return 0;
	} else {
		return dist;
	}
}

double twr_distance_calculation_dtu(uint32_t poll_rx_ts, uint32_t resp_tx_ts,
									uint32_t final_rx_ts, uint32_t Ra,
									uint32_t Da)
{
	/* 32 bit is enough for the timestamps themselves, but since we multiply
	 * them below we need a larger data type. Having one with 64 bit will
	 * automatically upgrade the others */
	int64_t Rb = final_rx_ts - resp_tx_ts;
	int64_t Db = resp_tx_ts - poll_rx_ts;

	/* This formula comes from the decawave documentation and source code
	 * A proof of it can be found here:
	 * https://forum.bitcraze.io/viewtopic.php?t=1944 */
	double tof_dtu = (double)(Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

	if (TWR_DEBUG_CALCULATION) {
		LOG_DBG("Poll RX TS:\t%lu", poll_rx_ts);
		LOG_DBG("Resp TX TS:\t%lu", resp_tx_ts);
		LOG_DBG("Fina RX TS:\t%lu", final_rx_ts);
		LOG_DBG("round1 (Ra)*:\t%lu", Ra);
		LOG_DBG("round2 (Rb):\t%lu", (uint32_t)Rb);
		LOG_DBG("reply1 (Da)*:\t%lu", Da);
		LOG_DBG("reply2 (Db):\t%lu", (uint32_t)Db);
		LOG_DBG("ToF DTU\t\t%d", (int)tof_dtu);
	} else if (tof_dtu < 0) {
		LOG_ERR("ToF DTU %d", (int)tof_dtu);
	}

	return tof_dtu;
}

int twr_distance_calculation(uint32_t poll_rx_ts, uint32_t resp_tx_ts,
							 uint32_t final_rx_ts, uint32_t Ra, uint32_t Da)
{
	double tof_dtu = twr_distance_calculation_dtu(poll_rx_ts, resp_tx_ts,
												  final_rx_ts, Ra, Da);
	double tof = DTU_TO_PS(tof_dtu);
	int dist = round(TIME_TO_DISTANCE(tof) * 100.0); // in cm, rounded

#if TWR_DEBUG_CALCULATION
	LOG_DBG("ToF PicoSec\t%d", (int)(tof * 1000000000000.0));
	LOG_DBG("Distance:\t%d cm", dist);
#endif
	return dist;
}

/*
 * Management
 */

static void twr_callback(uint64_t src, uint64_t dst, uint16_t dist,
						 uint16_t cnum)
{
	if (twr_observer_cb) {
		twr_observer_cb(src, dst, dist, cnum);
	}
}

static void twr_retry(void)
{
	if (++retry < TWR_MAX_RETRY) {
		int d = rand() % TWR_RETRY_DELAY;
		deca_sleep(d);
		LOG_INF("retry %d to " LADDR_FMT " after %d ms", retry,
				LADDR_PAR(twr_dst), d);
		twr_send_poll(twr_dst);
	} else {
		LOG_ERR("retry limit exceeded " LADDR_FMT, LADDR_PAR(twr_dst));
		twr_callback(twr_my_mac(twr_dst), twr_dst, TWR_FAILED_VALUE, twr_cnum);
		in_progress = false;
	}
}

static void twr_handle_timeout(uint32_t status)
{
	LOG_ERR("RX timeout from " LADDR_FMT, LADDR_PAR(twr_dst));
	if (expected_msg == TWR_MSG_RESP || expected_msg == TWR_MSG_REPO
		|| expected_msg == TWR_MSG_SSRESP) {
		/* The TAG (Initiator) side can retry the whole exchange */
		twr_retry();
	} else if (expected_msg == TWR_MSG_FINA) {
		/* The ANCOR (Passive) side can only log the error */
		LOG_ERR("RX timeout, did not receive final");
		// dev_update_state(current_anc_idx, TWR_FAIL);
		expected_msg = 0;
		in_progress = false;
	}
}

static void twr_handle_result(uint64_t src, uint64_t dst, uint16_t dist,
							  uint16_t cnum, bool reported, bool initiator)
{
	if (dist == TWR_FAILED_VALUE) {
		LOG_ERR("#%d " LADDR_FMT " -> " LADDR_FMT
				": Distance calculation failed %s",
				cnum, LADDR_PAR(src), LADDR_PAR(dst), reported ? "REP" : "");
		if (initiator) {
			twr_retry();
		}
	} else if (dist == 0) {
		// distance reported as 0, may be too close, or may be failed: retry
		LOG_INF("#%d " LADDR_FMT " -> " LADDR_FMT ": %u cm %s", cnum,
				LADDR_PAR(src), LADDR_PAR(dst), dist, reported ? "REP" : "");
		if (initiator) {
			twr_retry();
		}
	} else if (dist == TWR_OK_VALUE) {
		// this is on initiator side when no reports are expected, we don't know
		// the distance
		LOG_INF("#%d " LADDR_FMT " -> " LADDR_FMT ": OK", cnum, LADDR_PAR(src),
				LADDR_PAR(dst));
	} else {
		LOG_INF("#%d " LADDR_FMT " -> " LADDR_FMT ": %u cm %s", cnum,
				LADDR_PAR(src), LADDR_PAR(dst), dist, reported ? "REP" : "");
		twr_callback(src, dst, dist, cnum);
		in_progress = false;
	}
}

/*
 * Message handlers
 */

/* ANCOR */
static void twr_handle_final(const struct twr_msg_final* msg_final,
							 uint64_t final_rx_ts, uint64_t src)
{
	DBG_UWB("Received Final from " LADDR_FMT, LADDR_PAR(src));

	expected_msg = 0;
	uint32_t resp_tx_ts = dwt_readtxtimestamplo32();

	int dist
		= twr_distance_calculation(last_poll_rx_ts, resp_tx_ts, final_rx_ts,
								   msg_final->round, msg_final->delay);
	dist = twr_fixup_distance(dist);

#if TWR_SEND_REPORT
	// result will be handled after sending the report (time critical)
	twr_send_report(src, dist, msg_final->cnum, final_rx_ts);
#else
	// add result. we have been the destination of this TWR sequence
	twr_handle_result(src, twr_my_mac(src), dist, msg_final->cnum, false,
					  false);
#endif
}

/* TAG */
static void twr_handle_report(const struct twr_msg_report* msg, uint64_t src)
{
	/* no more messages expected */
	expected_msg = 0;

	/* distance back to me (tag) */
	twr_handle_result(twr_my_mac(src), src, msg->dist, msg->cnum, true, true);
}

static void twr_handle_ss_response(const struct rxbuf* rx, uint64_t src)
{
	const struct twr_msg_ss_resp* msg = dwprot_get_payload(rx->buf);
	uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
	uint32_t resp_rx_ts = (uint32_t)rx->ts;

	uint32_t rtd_init = resp_rx_ts - poll_tx_ts;
	uint32_t rtd_resp = msg->resp_tx_ts - msg->poll_rx_ts;

#if CONFIG_DECA_USE_CARRIERINTEG
	float clockOffsetRatio = -dwphy_get_rx_clock_offset_ci(rx->ci) / 1.0e6;
#else
	float clockOffsetRatio
		= ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
#endif

	double tof
		= DTU_TO_PS((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0);
	int dist = TIME_TO_DISTANCE(tof) * 100;

	dist = twr_fixup_distance(dist);
	twr_handle_result(twr_my_mac(src), src, dist, twr_cnum, false, true);
}

static size_t twr_get_msg_len(uint8_t func)
{
	switch (func) {
	case TWR_MSG_POLL:
		return 0;
	case TWR_MSG_RESP:
		return 0;
	case TWR_MSG_FINA:
		return sizeof(struct twr_msg_final);
	case TWR_MSG_REPO:
		return sizeof(struct twr_msg_report);
	case TWR_MSG_SSPOLL:
		return 0;
	case TWR_MSG_SSRESP:
		return sizeof(struct twr_msg_ss_resp);
	}
	return 0;
}

void twr_handle_message(const struct rxbuf* rx)
{
	uint64_t src = dwprot_get_src(rx->buf);
	uint8_t func = dwprot_get_func(rx->buf);

	/* drop unexpected messages, but always allow POLL in case the sender needs
	 * to retry */
	if (expected_msg != 0 && func != expected_msg && func != TWR_MSG_POLL) {
		LOG_ERR("Drop unexpected MSG %X from " LADDR_FMT, func, LADDR_PAR(src));
		return;
	}

	/* check length */
	if (dwprot_get_payload_len(rx->buf, rx->len) != twr_get_msg_len(func)) {
		LOG_ERR("Drop invalid length MSG %X from " LADDR_FMT, func,
				LADDR_PAR(src));
		return;
	}

	switch (func) {
	case TWR_MSG_POLL:
		twr_send_response(src, rx->ts);
		break;
	case TWR_MSG_RESP:
		twr_send_final(src, rx->ts);
		break;
	case TWR_MSG_FINA:
		twr_handle_final(dwprot_get_payload(rx->buf), rx->ts, src);
		break;
	case TWR_MSG_REPO:
		twr_handle_report(dwprot_get_payload(rx->buf), src);
		break;
	case TWR_MSG_SSPOLL:
		twr_send_ss_response(src, rx->ts);
		break;
	case TWR_MSG_SSRESP:
		twr_handle_ss_response(rx, src);
		break;
	default:
		LOG_ERR("Unknown MSG %X from " LADDR_FMT, func, LADDR_PAR(src));
	}
}

/*
 * API
 */

void twr_init(uint8_t rate_dw, uint8_t plen_dw, uint8_t prf_dw,
			  uint32_t processing_delay_us)
{
	/* Calculate reply delay: To make the ToF calculation error as small as
	 * possible in the presence of clock drift, the reply delays on both
	 * sides should be
	 * - as short as possible
	 * - as close as possible (small difference)
	 *
	 * For calculation of the biggest necessary delay in the TWR sequence
	 * we need to consider the time between RESP (with MAC_PROTO_MIN_LEN)
	 * has been received and FINAL can be sent.
	 */

	/* the processing time is a constant plus the time it takes to transfer
	 * the packet data over SPI */
	uint32_t proc_time_us
		= processing_delay_us + TWR_SPI_US_PER_BYTE * DWMAC_PROTO_SHORT_LEN
		  + TWR_SPI_US_PER_BYTE
				* (DWMAC_PROTO_SHORT_LEN + sizeof(struct twr_msg_final));

	/* TODO: CHECK: Processing time is higher when debugging is on */
#if CONFIG_DECA_DEBUG_IRQ_TIME || CONFIG_DECA_DEBUG_RX_DUMP                    \
	|| CONFIG_DECA_DEBUG_TX_DUMP || CONFIG_DECA_DEBUG_TX_TIME                  \
	|| CONFIG_DECA_DEBUG_RX_STATUS || CONFIG_DECA_READ_RXDIAG                  \
	|| TWR_DEBUG_CALCULATION
	proc_time_us += 400;
#endif

	/* Calculate delay from packet times
	 *
	 * From the RMARKER time (RX timestamp) we need
	 * 1) Time of PHY header
	 * 2) Time to receive data packet (RESP which is MAC_PROTO_MIN_LEN)
	 * 3) Processing time
	 * 4) Time for preamble + SFD until RMARKER of TX frame
	 *
	 * The first step is the packet times in picoseconds / 10, then we
	 * switch to microseconds */
	uint32_t twr_delay_us
		= dwphy_calc_phyhdr_time(rate_dw)
		  + dwphy_calc_data_time(rate_dw, DWMAC_PROTO_SHORT_LEN)
		  + dwphy_calc_preamble_time(plen_dw, prf_dw, rate_dw);
	twr_delay_us = PKTTIME_TO_USEC(twr_delay_us);
	twr_delay_us += proc_time_us;
	twr_delay_dtu = US_TO_DTU(twr_delay_us);

	/* RX delay is the time between the packet was sent completely (end of
	 * data) until we need to turn the RX on to receive the next packet.
	 * This is the processing time in UUS */
	twr_rx_delay = US_TO_UUS(proc_time_us);
	twr_pto = dwphy_get_recommended_preambletimeout();

#if CONFIG_DECA_DEBUG_IRQ_TIME || CONFIG_DECA_DEBUG_RX_DUMP                    \
	|| CONFIG_DECA_DEBUG_TX_DUMP || CONFIG_DECA_DEBUG_TX_TIME                  \
	|| CONFIG_DECA_DEBUG_RX_STATUS || CONFIG_DECA_READ_RXDIAG
	twr_pto += 3;
#endif

	LOG_INF("delay %ld us RX delay %ld us PTO %d (%d us)", twr_delay_us,
			proc_time_us, twr_pto, dwphy_pac_to_usec(twr_pto));

#if 0
	// distance formula test
	twr_distance_calculation(0xb2ad1f34, 0xf5155e34, 0x377d8aed,
				 0x42685421, 0x42684034);
	//-> expect 49 cm

	twr_distance_calculation(0xc786537d, 0x09ee9234, 0x4c56c3cf,
				 0x42684E28, 0x42684034);
	//-> expect 25 cm
#endif
}

bool twr_start(uint64_t dst)
{
	if (!dwhw_is_ready()) {
		return false;
	}

	twr_dst = dst;
	single_sided = false;
	twr_cnum++;
	in_progress = true;
	retry = 0;
	return twr_send_poll(twr_dst);
}

bool twr_start_ss(uint64_t dst)
{
	if (!dwhw_is_ready()) {
		return false;
	}

	twr_dst = dst;
	single_sided = true;
	twr_cnum++;
	in_progress = true;
	retry = 0;
	return twr_send_poll(twr_dst);
}

void twr_cancel(void)
{
	in_progress = false;
	retry = 0;
	expected_msg = 0;
}

void twr_set_observer(twr_cb_t cb)
{
	twr_observer_cb = cb;
}

bool twr_in_progress(void)
{
	return in_progress;
}

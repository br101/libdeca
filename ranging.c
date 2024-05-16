#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "dwhw.h"
#include "dwmac.h"
#include "dwphy.h"
#include "dwtime.h"
#include "dwutil.h"
#include "mac802154.h"
#include "ranging.h"

#include "log.h"

static const char* LOG_TAG = "TWR";

#define TWR_DEBUG_CALCULATION 0
#define TWR_MAX_RETRY		  3
#define TWR_RETRY_DELAY		  20 /* random with this maximum in ms */
#define TWR_SEND_REPORT		  1
#define TWR_SPI_US_PER_BYTE	  2.3 /* us measured with 8MHz DMA for 12 byte */
#define TWR_PROCESSING_TIME	  600 /* us */

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
	uint16_t cnum; // command number / TWR ID
} __attribute__((packed));

struct twr_msg_ss_resp {
	uint32_t poll_rx_ts;
	uint32_t resp_tx_ts;
} __attribute__((packed));

struct twr_msg_report {
	uint16_t cnum; // command number / TWR ID
	uint16_t dist;
} __attribute__((packed));

/* calculated in init */
static uint64_t twr_delay_dtu;
static uint32_t twr_rx_delay; // UUS
static uint16_t twr_pto;

/* state */
static bool single_sided = false;
static uint8_t expected_msg = 0;
static int retry = 0;
static bool in_progress = false;
static uint16_t twr_dst[TWR_MAX_DST];
static int current_idx = -1;	// index into list of destinations
static int twr_cnum = 0;		// command number or TWR sequence number
static struct twr_res* twr_res; // where to write result
static size_t twr_res_len;		// length of result
static void (*twr_done_handler)(int res); // called when TWR sequence done
static uint64_t last_poll_rx_ts;

static void twr_retry(void);
static bool twr_send_report(uint16_t tag, uint16_t dist, uint64_t final_rx_ts);
static void twr_sequence_next(void);
static void twr_write_result_buffer(int dist);
static uint16_t twr_fixup_distance(int dist);
static void twr_handle_result(uint16_t dist, uint16_t cnum, uint16_t src,
							  bool reported, bool move_on);

/*
 * Essential TWR messages
 */

/* TAG -> ANCOR */
static bool twr_send_poll(uint16_t ancor)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL)
		return false;

	dwmac_tx_prepare_prot(tx, 0, single_sided ? TWR_MSG_SSPOLL : TWR_MSG_POLL,
						  ancor);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_timeout_handler(tx, twr_handle_timeout);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Poll to " ADDR_FMT, ancor);
		expected_msg = single_sided ? TWR_MSG_SSRESP : TWR_MSG_RESP;
	} else {
		LOG_ERR("Failed to send Poll to " ADDR_FMT, ancor);
		twr_retry();
	}

	return res;
}

/* ANCOR -> TAG */
static bool twr_send_response(uint16_t tag, uint64_t poll_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	last_poll_rx_ts = poll_rx_ts;
	uint64_t resp_tx_time = poll_rx_ts + twr_delay_dtu;

	dwmac_tx_prepare_prot(tx, 0, TWR_MSG_RESP, tag);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_txtime(tx, resp_tx_time);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Response to " ADDR_FMT " after %dus", tag,
				(int)DTU_TO_US(resp_tx_time - poll_rx_ts));
		expected_msg = TWR_MSG_FINA;
	} else {
		LOG_ERR("Failed to send Response to " ADDR_FMT, tag);
		LOG_INF_TS("rx_ts", poll_rx_ts);
		LOG_INF_TS("tx_ts", resp_tx_time);
		LOG_INF_TS("delay", twr_delay_dtu);
		expected_msg = 0;
	}

	return res;
}

/* ANCOR -> TAG */
static bool twr_send_ss_response(uint16_t tag, uint64_t poll_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	/* Delayed TX time has a 8ns resolution because the last 9 bit of the
	 * DTU are ignored when programming the delayed TX time. We need to do
	 * the same in the calculated TX time */
	uint64_t resp_tx_time = (poll_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;

	dwmac_tx_prepare_prot(tx, sizeof(struct twr_msg_ss_resp), TWR_MSG_SSRESP,
						  tag);
	struct twr_msg_ss_resp* msg = (struct twr_msg_ss_resp*)tx->u.s.pbuf;
	msg->poll_rx_ts = (uint32_t)poll_rx_ts;
	msg->resp_tx_ts = (uint32_t)(resp_tx_time + DWPHY_ANTENNA_DELAY);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_set_txtime(tx, resp_tx_time);

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent SS Response to " ADDR_FMT " after %dus", tag,
				(int)DTU_TO_US(resp_tx_time - poll_rx_ts));
		LOG_DBG_TS("\tPoll RX TS:\t", poll_rx_ts);
		LOG_DBG_TS("\tResp TX TS:\t", resp_tx_time);
		expected_msg = 0;
	} else {
		LOG_ERR("Failed to send Response to " ADDR_FMT, tag);
		expected_msg = 0;
	}

	return res;
}

static void twr_handle_ss_response(const struct rxbuf* rx)
{
	struct twr_msg_ss_resp* msg = (struct twr_msg_ss_resp*)rx->u.s.pbuf;
	uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
	uint32_t resp_rx_ts = (uint32_t)rx->ts;

	uint32_t rtd_init = resp_rx_ts - poll_tx_ts;
	uint32_t rtd_resp = msg->resp_tx_ts - msg->poll_rx_ts;

#if DWMAC_USE_CARRIERINTEG
	float clockOffsetRatio = dwphy_get_rx_clock_offset_ci(rx->ci) / 1.0e6;
#else
	float clockOffsetRatio = 0;
#endif
	double tof
		= DTU_TO_PS((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0);
	int dist = TIME_TO_DISTANCE(tof) * 100;

	dist = twr_fixup_distance(dist);
	twr_handle_result(dist, twr_cnum, rx->u.s.hdr.src, false, true);
}

/* TAG -> ANCOR */
static bool twr_send_final(uint16_t ancor, uint64_t resp_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	uint64_t poll_tx_ts = deca_get_tx_timestamp();

	/* Delayed TX time has a 8ns resolution because the last 9 bit of the
	 * DTU are ignored when programming the delayed TX time. We need to do
	 * the same in the calculated TX time */
	uint64_t final_tx_time = (resp_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;

	/* Final TX timestamp is the transmission time we programmed plus the TX
	 * antenna delay. */
	uint64_t final_tx_ts = (final_tx_time + DWPHY_ANTENNA_DELAY) & DTU_MASK;

	struct twr_msg_final* final_msg = (struct twr_msg_final*)tx->u.s.pbuf;
	final_msg->cnum = twr_cnum;
	final_msg->round = resp_rx_ts - poll_tx_ts;
	final_msg->delay = final_tx_ts - resp_rx_ts;

	dwmac_tx_prepare_prot(tx, sizeof(struct twr_msg_final), TWR_MSG_FINA,
						  ancor);
	dwmac_tx_set_ranging(tx);
	dwmac_tx_set_txtime(tx, final_tx_time);
#if TWR_SEND_REPORT
	dwmac_tx_expect_response(tx, twr_rx_delay);
	dwmac_tx_set_preamble_timeout(tx, twr_pto);
	dwmac_tx_set_timeout_handler(tx, twr_handle_timeout);
#endif

	bool res = dwmac_tx_queue(tx);
	if (res) {
		DBG_UWB("Sent Final to " ADDR_FMT " after %dus", ancor,
				(int)DTU_TO_US(final_tx_time - resp_rx_ts));
		//  LOG_DBG_TS("\tPoll TX TS:\t", poll_tx_ts);
		//  LOG_DBG_TS("\tResp RX TS:\t", resp_rx_ts);
		//  LOG_DBG_TS("\tFina TX TS:\t", final_tx_time);
		expected_msg = TWR_SEND_REPORT ? TWR_MSG_REPO : 0;
	} else {
		LOG_ERR("Failed to send Final");
		twr_retry();
	}

#if !TWR_SEND_REPORT
	/* if reports are not sent by the other side, we assume everything is OK
	 * if the final message was sent. We don't know the distance, so we
	 * just record "OK" */
	twr_write_result_buffer(TWR_OK_VALUE);
	twr_sequence_next();
#endif

	return res;
}

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

static void twr_handle_result(uint16_t dist, uint16_t cnum, uint16_t src,
							  bool reported, bool move_on)
{
	if (reported) { // not on the "ancor" side
		twr_write_result_buffer(dist);
	}

	if (dist == TWR_FAILED_VALUE) {
		LOG_ERR("#%d " ADDR_FMT ": Distance calculation failed %s", cnum, src,
				reported ? "REP" : "");
		if (move_on) {
			twr_retry(); // may move to next in sequence
		}
	} else if (dist == 0) {
		// distance reported as 0, may be too close, or may be failed: retry
		LOG_INF("#%d " ADDR_FMT ": %u cm %s", cnum, src, dist,
				reported ? "REP" : "");
		if (move_on) {
			twr_retry(); // may move to next in sequence
		}
	} else {
		LOG_INF("#%d " ADDR_FMT ": %u cm %s", cnum, src, dist,
				reported ? "REP" : "");
		if (move_on) {
			twr_sequence_next();
		}
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

/* ANCOR */
static void twr_handle_final(const union macbuf* mb, uint64_t final_rx_ts,
							 size_t len)
{
	expected_msg = 0;

	if (len - DWMAC_PROTO_MIN_LEN != sizeof(struct twr_msg_final)) {
		LOG_ERR("Unexpected final msg len");
		return;
	}

	DBG_UWB("Received Final from " ADDR_FMT, mb->s.hdr.src);

	struct twr_msg_final* msg_final = (struct twr_msg_final*)mb->s.pbuf;
	uint32_t resp_tx_ts = dwt_readtxtimestamplo32();

	int dist
		= twr_distance_calculation(last_poll_rx_ts, resp_tx_ts, final_rx_ts,
								   msg_final->round, msg_final->delay);
	dist = twr_fixup_distance(dist);
	twr_handle_result(dist, msg_final->cnum, mb->s.hdr.src, false, false);

#if TWR_SEND_REPORT
	twr_send_report(mb->s.hdr.src, dist, final_rx_ts);
#endif
}

static void twr_write_result_buffer(int dist)
{
	if (current_idx < 0 || current_idx >= TWR_MAX_DST) {
		LOG_ERR("res index out of range");
		return;
	}

	if ((current_idx + 1) * sizeof(struct twr_res) > twr_res_len) {
		LOG_ERR("result buffer too small %d vs %d",
				(current_idx + 1) * sizeof(struct twr_res), twr_res_len);
		return;
	}

	uint16_t addr = twr_dst[current_idx];
	if (addr == 0) {
		LOG_ERR("res invalid index %d", current_idx);
		twr_cancel();
		return;
	}

	twr_res[current_idx].addr = addr;
	twr_res[current_idx].dist = dist;
}

static void twr_retry(void)
{
	if (current_idx < 0 || current_idx >= TWR_MAX_DST) {
		LOG_ERR("retry index out of range");
		return;
	}

	uint16_t addr = twr_dst[current_idx];
	if (addr == 0) {
		LOG_ERR("retry invalid index %d", current_idx);
		twr_cancel();
		return;
	}

	if (++retry < TWR_MAX_RETRY) {
		int d = rand() % TWR_RETRY_DELAY;
		deca_sleep(d);
		LOG_INF("retry %d to " ADDR_FMT " after %d ms", retry, addr, d);
		twr_send_poll(addr);
	} else {
		LOG_ERR("retry limit exceeded " ADDR_FMT, addr);
		twr_write_result_buffer(TWR_FAILED_VALUE);
		twr_sequence_next();
	}
}

bool twr_start(const uint16_t anc[], size_t len, uint16_t cnum, void* res,
			   size_t res_len, void (*done_handler)(int res), bool ss)
{
	if (!dwhw_is_ready()) {
		return false;
	}

	/* first add tentative ancor to our list */
	size_t j = 0;
	for (size_t i = 0; i < TWR_MAX_DST && i < len; i++) {
		if (anc[i] == 0)
			break;
		if (anc[i] == 0xffff || anc[i] == dwmac_get_mac16()) {
			LOG_ERR("Ignoring invalid dst " ADDR_FMT, anc[i]);
			continue;
		}
		twr_dst[j++] = anc[i];
	}

	if (j == 0) {
		return false;
	}

	while (j < TWR_MAX_DST) {
		twr_dst[j++] = 0; // end marker
	}

	// last_batt = tag_get_battery();

	/* Immediately start TWR sequence to first ancor */
	in_progress = true;
	current_idx = 0;
	retry = 0;
	twr_cnum = cnum;
	twr_res = res;
	twr_res_len = res_len;
	twr_done_handler = done_handler;
	single_sided = ss;
	twr_send_poll(twr_dst[0]);
	return true;
}

/*
 * Report messages
 */

/* ANCOR -> TAG */
static bool twr_send_report(uint16_t tag, uint16_t dist, uint64_t final_rx_ts)
{
	struct txbuf* tx = dwmac_txbuf_get();
	if (tx == NULL) {
		return false;
	}

	struct twr_msg_report* msg = (struct twr_msg_report*)tx->u.s.pbuf;
	msg->cnum = twr_cnum;
	msg->dist = dist;

	expected_msg = 0;

	dwmac_tx_prepare_prot(tx, sizeof(struct twr_msg_report), TWR_MSG_REPO, tag);
	uint64_t rep_tx_time = (final_rx_ts + twr_delay_dtu) & DTU_DELAYEDTRX_MASK;
	dwmac_tx_set_txtime(tx, rep_tx_time);

	bool res = dwmac_tx_queue(tx);
	LOG_TX_RES(res, "Report to " ADDR_FMT ": distance %u cm", tag, dist);
	return res;
}

/* TAG or MASTER */
static void twr_handle_report(const struct rxbuf* rx)
{
	if (rx->len - DWMAC_PROTO_MIN_LEN != sizeof(struct twr_msg_report)) {
		LOG_ERR("Unexpected report msg len");
		return;
	}

	const struct twr_msg_report* msg
		= (const struct twr_msg_report*)rx->u.s.pbuf;

	/* no more messages expected */
	expected_msg = 0;

	/* single distance back to me (tag) */
	twr_handle_result(msg->dist, twr_cnum, rx->u.s.hdr.src, true, true);
}

/*
 * sequence functions
 */

/* TAG */
static void twr_sequence_next(void)
{
	if (current_idx < 0 || current_idx >= TWR_MAX_DST) {
		LOG_ERR("next index out of range");
		return;
	}

	current_idx++;
	if (current_idx < TWR_MAX_DST && twr_dst[current_idx] != 0) {
		/* move to next */
		uint16_t addr = twr_dst[current_idx];
		if (addr == 0) {
			LOG_ERR("next invalid index %d", current_idx);
			twr_cancel();
			return;
		}

		retry = 0;
		twr_send_poll(addr);
	} else {
		/* done with all */
		if (twr_done_handler != NULL) {
			/* result len has to be a multiple of twr_res and smaller than total
			 */
			size_t i = 1;
			while (i * sizeof(struct twr_res) <= twr_res_len
				   && i <= (size_t)current_idx) {
				i++;
			}

			twr_done_handler((i - 1) * sizeof(struct twr_res));
		}

		in_progress = false;
		twr_cnum = 0;
		current_idx = -1;
	}
}

/*
 * MAC layer / protocol callbacks
 */

void twr_handle_message(struct rxbuf* rx)
{
	/* drop unexpected messages, but always allow POLL in case the sender needs
	 * to retry */
	if (expected_msg != 0 && rx->u.s.func != expected_msg
		&& rx->u.s.func != TWR_MSG_POLL) {
		LOG_ERR("Drop unexpected MSG %X from " ADDR_FMT, rx->u.s.func,
				rx->u.s.hdr.src);
		return;
	}

	switch (rx->u.s.func) {
	case TWR_MSG_POLL:
		twr_send_response(rx->u.s.hdr.src, rx->ts);
		break;
	case TWR_MSG_RESP:
		twr_send_final(rx->u.s.hdr.src, rx->ts);
		break;
	case TWR_MSG_FINA:
		twr_handle_final(&rx->u, rx->ts, rx->len);
		break;
	case TWR_MSG_REPO:
		twr_handle_report(rx);
		break;
	case TWR_MSG_SSPOLL:
		twr_send_ss_response(rx->u.s.hdr.src, rx->ts);
		break;
	case TWR_MSG_SSRESP:
		twr_handle_ss_response(rx);
		break;
	default:
		LOG_ERR("Unknown MSG %X from " ADDR_FMT, rx->u.s.func, rx->u.s.hdr.src);
	}
}

void twr_handle_timeout(uint32_t status)
{
	if (current_idx < 0 || current_idx >= TWR_MAX_DST) {
		LOG_ERR("timeout index out of range");
		return;
	}

	uint16_t addr = twr_dst[current_idx];
	if (addr == 0) {
		LOG_ERR("timeout invalid index %d", current_idx);
		twr_cancel();
		return;
	}

	LOG_ERR("RX timeout from " ADDR_FMT, addr);
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

bool twr_in_progress(void)
{
	return in_progress;
}

void twr_init(uint8_t rate_dw, uint8_t plen_dw, uint8_t prf_dw)
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
		= TWR_PROCESSING_TIME + TWR_SPI_US_PER_BYTE * DWMAC_PROTO_MIN_LEN
		  + TWR_SPI_US_PER_BYTE
				* (DWMAC_PROTO_MIN_LEN + sizeof(struct twr_msg_final));

	/* TODO: CHECK: Processing time is higher when debugging is on */
#if CONFIG_DECA_DEBUG_IRQ_TIME || CONFIG_DECA_DEBUG_RX_DUMP                    \
	|| CONFIG_DECA_DEBUG_TX_DUMP || CONFIG_DECA_DEBUG_TX_TIME                  \
	|| CONFIG_DECA_DEBUG_RX_STATUS || DWMAC_INCLUDE_RXDIAG                     \
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
		  + dwphy_calc_data_time(rate_dw, DWMAC_PROTO_MIN_LEN)
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
	|| CONFIG_DECA_DEBUG_RX_STATUS || DWMAC_INCLUDE_RXDIAG
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

void twr_cancel(void)
{
	in_progress = false;
	twr_cnum = 0;
	current_idx = -1;
	retry = 0;
	expected_msg = 0;
}

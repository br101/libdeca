/*
 * libdeca - UWB Library for Qorvo/Decawave DW3000
 *
 * Copyright (C) 2016 - 2024 Bruno Randolf (br@einfach.org)
 *
 * This source code is licensed under the GNU Lesser General Public License,
 * Version 3. See the file LICENSE.txt for more details.
 */

#include <stdlib.h> // abs

#include <deca_device_api.h>
#include <deca_version.h>
#ifdef DW3000_DRIVER_VERSION // == 0x040000
#include <deca_regs.h>
#endif

#include "dwphy.h"
#include "dwproto.h"

#include "log.h"
#define DWPHY_PRF			DWT_PRF_64M
#define TEST_EXAMPLE_CONFIG 0

#ifndef __ZEPHYR__
static const char* LOG_TAG = "DECA";
#endif

#if TEST_EXAMPLE_CONFIG
// this is the config used in many decadriver examples
static dwt_config_t config = {
	.chan = 5,
	.txPreambLength = DWT_PLEN_128,
	.rxPAC = DWT_PAC8,
	.txCode = 9,
	.rxCode = 9,
	.sfdType = DWT_SFD_DW_8,
	.dataRate = DWT_BR_6M8,		/* Data rate. */
	.phrMode = DWT_PHRMODE_STD, /* PHY header mode. */
	.phrRate = DWT_PHRRATE_STD, /* PHY header rate. */
	.sfdTO = (129 + 8 - 8),		/* (plen + 1 + SFD length - PAC size) */
	.stsMode = DWT_STS_MODE_OFF,
	.stsLength = DWT_STS_LEN_64,
	.pdoaMode = DWT_PDOA_M0 /* off */
};
#else
// default config
static dwt_config_t config = {
	.chan = 9,
	.txPreambLength = DWT_PLEN_64,
	.rxPAC = DWT_PAC8,
	.txCode = 11,
	.rxCode = 11,
	.sfdType = DWT_SFD_IEEE_4Z,
	.dataRate = DWT_BR_6M8,
	.phrMode = DWT_PHRMODE_STD,
	.phrRate = DWT_PHRRATE_STD,
	.sfdTO = (64 + 1 + 8 - 8),	 /* (plen + 1 + SFD length - PAC size) */
	.stsMode = DWT_STS_MODE_OFF, // DWT_STS_MODE_1 | DWT_STS_MODE_SDC,
	.stsLength = DWT_STS_LEN_64,
	.pdoaMode = DWT_PDOA_M0, /* off */
};
#endif

/*
 * TX Power Configuration Settings
 */
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and
 * power of the spectrum at the current temperature. These values can be
 * calibrated prior to taking reference measurements. */
dwt_txconfig_t txconfig_ch5 = {
	0x34,		/* PG delay */
	0xfdfdfdfd, /* TX power */
	0x0			/* PG count */
};

dwt_txconfig_t txconfig_ch9 = {
	0x34,		/* PG delay */
	0xfefefefe, /* TX power */
	0x0			/* PG count */
};

static uint8_t phy_get_recommended_pac(uint16_t plen)
{
	/* TODO: check following comment from forum "with a preamble length 256
	 * a PAC size of 8 worked better than PAC size of 16" */
	switch (plen) {
	case DWT_PLEN_32:
	case DWT_PLEN_64:
	case DWT_PLEN_72:
	case DWT_PLEN_128:
		return DWT_PAC8;
	case DWT_PLEN_256:
		return DWT_PAC16;
	case DWT_PLEN_512:
		return DWT_PAC32;
	case DWT_PLEN_1024:
	case DWT_PLEN_1536:
	case DWT_PLEN_2048:
	case DWT_PLEN_4096:
		return DWT_PAC32;
	}

	LOG_ERR("Unknown PAC for PLEN %d", plen);
	return DWT_PAC32;
}

static bool check_preamble_len(uint8_t plen, uint8_t rate)
{
	/* check for recommended preamble lengths, return true if OK */
	return (rate == DWT_BR_6M8
			&& (plen == DWT_PLEN_64 || plen == DWT_PLEN_128
				|| plen == DWT_PLEN_256))
		   || (rate == DWT_BR_850K
			   && (plen == DWT_PLEN_256 || plen == DWT_PLEN_512
				   || plen == DWT_PLEN_1024));
}

static int phy_sfd_len(uint8_t rate, bool standard)
{
	/* Non-standard SFD len is 64, 16 or 8 for rate 110K, 850K or 6.8M.
	 * Standard defines SFD len of 64 for 110K and 8 for both other rates. */
	switch (rate) {
	case DWT_BR_850K:
		return standard ? 8 : 16;
	case DWT_BR_6M8:
		return 8;
	}
	return -1;
}

static int phy_calc_sfd_timeout(uint8_t plen, uint8_t pac, uint8_t rate)
{
	/* (plen + 1 + SFD length - PAC size) */
	return dwphy_plen_int(plen) + 1 + phy_sfd_len(rate, false)
		   - dwphy_pac_int(pac);
}

bool dwphy_config(void)
{
	if (config.sfdTO == 0) {
		config.sfdTO = phy_calc_sfd_timeout(config.txPreambLength, config.rxPAC,
											config.dataRate);
		LOG_INF("calculated SFD timeout: %d", config.sfdTO);
	} else {
		uint16_t sfdto = phy_calc_sfd_timeout(config.txPreambLength,
											  config.rxPAC, config.dataRate);
		if (config.sfdTO != sfdto) {
			LOG_WARN("SFD timeout may be wrong: %d vs %d", config.sfdTO, sfdto);
		}
	}

	int prf = DWPHY_PRF;

	LOG_INF("config CH %d %s PRF %dMHz Plen %d PAC %d", config.chan,
			dwphy_rate_str(config.dataRate), dwphy_prf_int(prf),
			dwphy_plen_int(config.txPreambLength), dwphy_pac_int(config.rxPAC));
	LOG_INF("config code %d/%d SFD %d", config.txCode, config.rxCode,
			config.sfdTO);
	LOG_INF("PRE %" PRIu32 " (SFD %" PRIu32 ") PHD %" PRIu32
			" %dB DATA %d = %d us",
			PKTTIME_TO_USEC(dwphy_calc_preamble_time(config.txPreambLength, prf,
													 config.dataRate)
							- dwphy_calc_sfd_time(prf, config.dataRate)),
			PKTTIME_TO_USEC(dwphy_calc_sfd_time(prf, config.dataRate)),
			PKTTIME_TO_USEC(dwphy_calc_phyhdr_time(config.dataRate)),
			DWMAC_PROTO_SHORT_LEN,
			(int)PKTTIME_TO_USEC(
				dwphy_calc_data_time(config.dataRate, DWMAC_PROTO_SHORT_LEN)),
			(int)PKTTIME_TO_USEC(
				dwphy_calc_packet_time(config.dataRate, config.txPreambLength,
									   prf, DWMAC_PROTO_SHORT_LEN)));

	if (!check_preamble_len(config.txPreambLength, config.dataRate)) {
		LOG_ERR("preamble length out of recommended range!");
	}
	if (config.rxPAC != phy_get_recommended_pac(config.txPreambLength)) {
		LOG_ERR("PAC out of recommended range!");
	}

	dwt_configure(&config);
	if (config.chan == 9) {
		dwt_configuretxrf(&txconfig_ch9);
	} else {
		dwt_configuretxrf(&txconfig_ch5);
	}

	dwt_setrxantennadelay(0);
	dwt_settxantennadelay(0);

	/* activate this for RX/TX timing debugging via GPIO5/6 */
	// dwt_setfinegraintxseq(0);
	// dwt_setlnapamode(1, 1);

	return true;
}

void dwphy_set_antenna_delay(uint16_t antdelay)
{
	dwt_setrxantennadelay(antdelay);
	dwt_settxantennadelay(antdelay);
}

const char* dwphy_rate_str(uint8_t br)
{
	switch (br) {
	case DWT_BR_850K:
		return "850Kbps";
	case DWT_BR_6M8:
		return "6.8Mbps";
	case DWT_BR_NODATA:
		return "(nodata)";
	}
	return "???";
}

/* rate in Kbps */
int dwphy_rate_int(uint8_t br)
{
	switch (br) {
	case DWT_BR_850K:
		return 850;
	case DWT_BR_6M8:
		return 6800;
	case DWT_BR_NODATA:
		return 0;
	}
	return -1;
}

int dwphy_plen_int(uint16_t pl)
{
	switch (pl) {
	case DWT_PLEN_4096:
		return 4096;
	case DWT_PLEN_2048:
		return 2048;
	case DWT_PLEN_1536:
		return 1536;
	case DWT_PLEN_1024:
		return 1024;
	case DWT_PLEN_512:
		return 512;
	case DWT_PLEN_256:
		return 256;
	case DWT_PLEN_128:
		return 128;
	case DWT_PLEN_64:
		return 64;
	case DWT_PLEN_32:
		return 32;
	case DWT_PLEN_72:
		return 72;
	}
	return -1;
}

int dwphy_pac_int(uint8_t p)
{
	switch (p) {
	case DWT_PAC32:
		return 32;
	case DWT_PAC16:
		return 16;
	case DWT_PAC8:
		return 8;
	case DWT_PAC4:
		return 4;
	}
	return -1;
}

int dwphy_prf_int(uint8_t prf)
{
	switch (prf) {
	case DWT_PRF_16M:
		return 16;
	case DWT_PRF_64M:
		return 64;
	case DWT_PRF_SCP:
		return 100; // TODO
	}
	return -1;
}

/** returns time of synchronization header SHR (preamble + SFD) in picoseconds /
 * 10 */
uint32_t dwphy_calc_preamble_time(uint8_t plen_dwt, uint8_t prf_dwt,
								  uint8_t rate_dwt)
{
	/* preamble symbol duration in ns from User Manual:
	 * PRF 16MHz: 993.59, PRF 64MHz: 1017.63 */
	uint32_t plen = dwphy_plen_int(plen_dwt);
	plen += phy_sfd_len(rate_dwt, false);
	if (prf_dwt == DWT_PRF_16M) {
		return plen * 99359;
	} else {
		return plen * 101763;
	}
}

/** returns time of SFD (it is included in preamble time) in picoseconds / 10 */
uint32_t dwphy_calc_sfd_time(uint8_t prf_dwt, uint8_t rate_dwt)
{
	/* preamble symbol duration in ns from User Manual:
	 * PRF 16MHz: 993.59, PRF 64MHz: 1017.63 */
	float plen = phy_sfd_len(rate_dwt, false);
	if (prf_dwt == DWT_PRF_16M) {
		return plen * 99359;
	} else {
		return plen * 101763;
	}
}

/** returns time of data part in picoseconds / 10 */
uint64_t dwphy_calc_data_time(uint8_t rate_dwt, int len)
{
	/* length in bits */
	len *= 8;

	/* frame includes 48 Reed-Solomon parity bits following each block
	 * of 330 data bits (or less) */
	len += CEIL_DIV(len, 330) * 48;

	/* multiply with symbol durations from DW1000 Datasheet, pg.18
	 * durations are in 100 * nanoseconds to make integer */
	switch (rate_dwt) {
	case DWT_BR_850K:
		return (uint64_t)len * 102564;
	case DWT_BR_6M8:
		return (uint64_t)len * 12821;
	}

	return 0;
}

/** returns time of PHY header (PHR) in picoseconds / 10 */
uint32_t dwphy_calc_phyhdr_time(uint8_t rate_dwt)
{
	/* calculate PHR time by multiplying bits with symbol time as in
	 * datasheet pg.18 and EVM code. Also see User Manual, pg. 203.
	 * The PHY header has 19 bits, but physically also includes 2 tail bits
	 * so in sum they are 21 bits. See IEEE 802.15.4-2001 pg. 195 and also
	 * the source code of EVM.
	 */
	return 21 * 102564;
}

/** returns time of complete frame time in picoseconds / 10 */
uint64_t dwphy_calc_packet_time(uint8_t rate_dwt, uint8_t plen_dwt,
								uint8_t prf_dwt, int data_len)
{
	return dwphy_calc_preamble_time(plen_dwt, prf_dwt, rate_dwt)
		   + dwphy_calc_phyhdr_time(rate_dwt)
		   + dwphy_calc_data_time(rate_dwt, data_len);
}

void dwphy_print_packet_times(void)
{
#if 0
	uint8_t plen_dwt[]
		= {DWT_PLEN_4096, DWT_PLEN_2048, DWT_PLEN_1536, DWT_PLEN_1024,
		   DWT_PLEN_512,  DWT_PLEN_256,	 DWT_PLEN_128,	DWT_PLEN_64};

	LOG_INF("Preamble time in us:");
	LOG_INF("        110K        850K        6.8M");
	LOG_INF("PLEN PRF16 PRF64 PRF16 PRF64 PRF16 PRF64");
	for (size_t i = 0; i < sizeof(plen_dwt); i++) {
		LOG_INF("%-4d %5lu %5lu %5lu %5lu",
				dwphy_plen_int(plen_dwt[i]),
				dwphy_calc_preamble_time(plen_dwt[i], DWT_PRF_16M, DWT_BR_850K),
				dwphy_calc_preamble_time(plen_dwt[i], DWT_PRF_64M, DWT_BR_850K),
				dwphy_calc_preamble_time(plen_dwt[i], DWT_PRF_16M, DWT_BR_6M8),
				dwphy_calc_preamble_time(plen_dwt[i], DWT_PRF_64M, DWT_BR_6M8));
	}

	uint8_t br_dwt[] = {DWT_BR_850K, DWT_BR_6M8};
	LOG_INF("Data time in us:");
	LOG_INF("RATE\t10B\t20B\t40B\t60B\t127B");
	for (size_t i = 0; i < sizeof(br_dwt); i++) {
		LOG_INF("%s\t%llu\t%llu\t%llu\t%llu\t%llu", dwphy_rate_str(br_dwt[i]),
				dwphy_calc_data_time(br_dwt[i], 10),
				dwphy_calc_data_time(br_dwt[i], 20),
				dwphy_calc_data_time(br_dwt[i], 40),
				dwphy_calc_data_time(br_dwt[i], 60),
				dwphy_calc_data_time(br_dwt[i], 127));
	}
#endif
}

/* positive value means the local RX clock is running slower than the remote TX
 * device */
float dwphy_get_rx_clock_offset_ci(int32_t ci)
{
	float clockOffsetHertz;

	// convert carrier integrator to clock offset in Hz.
	clockOffsetHertz = ci * FREQ_OFFSET_MULTIPLIER;

	switch (config.chan) {
	case 5:
		return clockOffsetHertz * (float)HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
	case 9:
		return clockOffsetHertz * (float)HERTZ_TO_PPM_MULTIPLIER_CHAN_9;
	default:
		LOG_ERR("Unknown Channel %d", config.chan);
		break;
	}
	return 0.0;
}

int dwphy_get_recommended_preambletimeout(void)
{
	int plen = dwphy_plen_int(config.txPreambLength);
	int pac = dwphy_pac_int(config.rxPAC);
	return plen / pac;
}

/** number of PAC for x microseconds depending on config */
int dwphy_usec_to_pac(uint32_t us)
{
	/* number of PAC in whole preamble */
	int num_pac
		= dwphy_plen_int(config.txPreambLength) / dwphy_pac_int(config.rxPAC);
	/* time of preamble */
	uint32_t pac_time = dwphy_calc_preamble_time(
		config.txPreambLength, 2 /*config.prf*/, config.dataRate);
	pac_time -= dwphy_calc_sfd_time(2 /*config.prf*/, config.dataRate);

	/* time of one PAC */
	pac_time /= num_pac;

	/* numer of PAC rounded up */
	uint64_t x = (uint64_t)us * 100000;
	return CEIL_DIV(x, pac_time);
}

/** PAC units to microseconds depending on config */
int dwphy_pac_to_usec(uint16_t pacs)
{
	/* number of PAC in whole preamble */
	int num_pac
		= dwphy_plen_int(config.txPreambLength) / dwphy_pac_int(config.rxPAC);
	/* time of preamble */
	uint64_t pac_time = dwphy_calc_preamble_time(
		config.txPreambLength, 2 /*config.prf*/, config.dataRate);
	pac_time -= dwphy_calc_sfd_time(2 /*config.prf*/, config.dataRate);

	/* time of one PAC */
	pac_time /= num_pac;

	/* time of x PAC */
	return PKTTIME_TO_USEC(pac_time * pacs);
}

/* Minimal/maximal value of the target XTAL offset in hundreds of PPM
 * (i.e. 1ppm = 100, 10ppm = 1000) */
#define XTAL_OFFSET_PPHM_MAX 100

/* The typical trimming range of DW3000 (with 2pF external caps is ~48ppm
 * (-30ppm to +18ppm) over all steps */
#define XTAL_AVG_TRIM_PER_PPHM                                                 \
	((XTAL_TRIM_BIT_MASK + 1) / 48.0f / 100) /* Trimming per 1 pphm */

static uint8_t xtalTrim;

void dwphy_xtal_trim(void)
{
	int16_t off_hw = dwt_readclockoffset();
	int off_pphm = (float)off_hw * (float)CLOCK_OFFSET_PPM_TO_RATIO * 1e6f * 100.0f;

	unsigned int off_abs = abs(off_pphm);

	if (off_abs > XTAL_OFFSET_PPHM_MAX) {
		int8_t tmp = xtalTrim;
		tmp -= (XTAL_OFFSET_PPHM_MAX / 2 + off_pphm) * XTAL_AVG_TRIM_PER_PPHM;

		if (tmp > XTAL_TRIM_BIT_MASK) {
			tmp = XTAL_TRIM_BIT_MASK;
		} else if (tmp < 0) {
			tmp = 0;
		}

		if (tmp == 0 || tmp != xtalTrim) {
			xtalTrim = tmp;
			dwt_setxtaltrim(xtalTrim);
			// LOG_INF("Set XTAL trim %d", xtalTrim);
		}
	}
}

void dwphy_set_rate(uint8_t br)
{
	config.dataRate = br;
}

uint8_t dwphy_get_rate(void)
{
	return config.dataRate;
}

void dwphy_set_plen(uint8_t plen)
{
	config.txPreambLength = plen;
}

uint8_t dwphy_get_plen(void)
{
	return config.txPreambLength;
}

uint8_t dwphy_get_prf(void)
{
	return DWPHY_PRF;
}

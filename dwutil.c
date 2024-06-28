#include "dwutil.h"

uint64_t mac_to_eui64(const uint8_t* mac)
{
	uint64_t eui64;
	uint8_t* m64 = (uint8_t*)&eui64;
	m64[0] = mac[0];
	m64[1] = mac[1];
	m64[2] = mac[2];
	// m64[3] = 0xFF;
	// m64[4] = 0xFE;
	m64[3] = mac[3];
	m64[4] = mac[4];
	m64[5] = mac[5];
	m64[6] = 0;
	m64[7] = 0;
	return eui64;
}

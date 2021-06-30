#include "exchenge.h"
#include "nrf_log.h"

uint16_t SendData(gSendFn DoSend, uint16_t mtu, uint8_t* data, uint16_t len)
{
	uint16_t sent = 0;
	uint16_t curr_count = 0;

	NRF_LOG_DEBUG("MTU: %d", mtu);

	while(sent < len)
	{
		curr_count = (sent + mtu> len) ? (len - sent) : mtu;

		DoSend( data+sent, curr_count );

		sent += curr_count;
		NRF_LOG_DEBUG("current: %d", curr_count);
	}
return sent;
}

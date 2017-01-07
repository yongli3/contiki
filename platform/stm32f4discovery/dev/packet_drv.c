#include <netconfig.h>
#include <contiki-net.h>
#include <debug-uart.h>
#include <stdio.h>

PROCESS(stm32f4discovery_packet_drv, "STM32F4Discovery packet driver process");

uint8_t stm32f4discovery_packet_drv_output()
{
	net_send(uip_buf, uip_len);

	return 0;
}

static void
stm32f4discovery_packet_drv_pollhandler(void)
{
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
				clock_time_t delay_start;

	uip_len = net_receive(&uip_buf);

	if (uip_len > 0)
	{
		//printf("Packet received, length %d, processing... ", uip_len);
		if (BUF->type == UIP_HTONS(UIP_ETHTYPE_ARP))
		{
			//printf("arp... ");
			uip_arp_arpin();

			if (uip_len > 0)
			{
				//printf("sending arp response.");
				net_send(uip_buf, uip_len);
			}
		}
		else if (BUF->type == UIP_HTONS(UIP_ETHTYPE_IP))
		{
						int i = 0;
			//printf("ip... ");
			uip_arp_ipin();
			uip_input();

			if (uip_len > 0)
			{
				//printf("sending response.");
				delay_start = clock_time();


				uip_arp_out();
				net_send(uip_buf, uip_len);
			}
		}
		else
		{
			printf("unknown eth type (%x), dropped.", UIP_HTONS(BUF->type));
		}

		//printf("\n");
	}

	process_poll(&stm32f4discovery_packet_drv);
}

PROCESS_THREAD(stm32f4discovery_packet_drv, ev, data)
{
	PROCESS_POLLHANDLER(stm32f4discovery_packet_drv_pollhandler());

	PROCESS_BEGIN();

	net_init();

	printf("Ethernet initialized.\n");

	tcpip_set_outputfunc(stm32f4discovery_packet_drv_output);

	process_poll(&stm32f4discovery_packet_drv);

	PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXIT);

	PROCESS_END();
}

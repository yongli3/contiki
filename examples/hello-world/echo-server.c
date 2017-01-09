#include <contiki.h>
#include <contiki-net.h>

#include <string.h>

static struct psock echo_ps;

static uint8_t buffer[50];

static PT_THREAD(handle_connection(struct psock *ps))
{
	PSOCK_BEGIN(ps);

	PSOCK_READTO(ps, '\n');

	PSOCK_SEND(ps, buffer, PSOCK_DATALEN(ps));

	PSOCK_CLOSE(ps);

	PSOCK_END(ps);
}

PROCESS(echo_server_process, "Echo server");

PROCESS_THREAD(echo_server_process, ev, data)
{
	PROCESS_BEGIN();

	tcp_listen(UIP_HTONS(7));

	while (1)
	{
		PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

		if (uip_connected())
		{
			PSOCK_INIT(&echo_ps, buffer, sizeof(buffer));

			while (!(uip_aborted() || uip_closed() || uip_timedout()))
			{
				PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

				handle_connection(&echo_ps);
			}
		}
	}

	PROCESS_END();
}

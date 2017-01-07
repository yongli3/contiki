/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include <contiki-conf.h>
#include "contiki.h"
#include <sys/clock.h>
#include <dev/leds.h>
#include <sys/rtimer.h>

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
PROCESS(hello_world_process2, "Hello world process 2");
AUTOSTART_PROCESSES(&hello_world_process, &hello_world_process2);//, &echo_server_process, &sensors_test_process, &rest_server);
/*---------------------------------------------------------------------------*/
static struct etimer timer;
static struct etimer timer2;
static struct rtimer task;
static struct rtimer task2;
static int counter = 0;
static int counter2 = 0;

static void
callback(struct rtimer *t, void *ptr, int status)
{
	rtimer_set(&task, RTIMER_NOW() + RTIMER_SECOND,  0, callback, NULL);

	printf("In rtimer\n");
}

static void
callback_once(struct rtimer *t, void *ptr, int status)
{
	printf("In callback_once\n");
}

PROCESS_THREAD(hello_world_process2, ev, data)
{
    printf("+%s\n\r", __func__);
    PROCESS_BEGIN();

    printf("Hello world from process_2\n");

		rtimer_init();
		rtimer_set(&task2, RTIMER_NOW() + RTIMER_SECOND *1, 0 , callback_once, NULL);
		rtimer_set(&task, RTIMER_NOW() + RTIMER_SECOND * 2,  0, callback, NULL);

    etimer_set(&timer2, 25);

    while (1)
    {
        PROCESS_WAIT_EVENT();

        if (ev == PROCESS_EVENT_TIMER)
        {
            leds_toggle(LEDS_GREEN);

            counter2 += 1;
            counter2 %= 2;

            etimer_reset(&timer2);
        }
    }

    PROCESS_END();
}
PROCESS_THREAD(hello_world_process, ev, data)
{
    printf("+%s\n\r", __func__);

  PROCESS_BEGIN();

  printf("Hello world from process_1\n");

  //adc_init();

  etimer_set(&timer, 100);


    while (1)
    {
      PROCESS_WAIT_EVENT();

      if (ev == PROCESS_EVENT_TIMER)
      {
				if (counter %2 == 0)
				{
					leds_on(LEDS_BLUE);
				}
				else
				{
					leds_off(LEDS_BLUE);
				}

				counter += 1;
				counter %= 4;

	    		
		    //printf("%d\n", adc_get_value() * 2900 / 0xfff );
		    //last_adc_value = adc_get_value();



        etimer_reset(&timer);
      }
    }

  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

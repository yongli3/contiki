/*
 * Copyright (c) 2008, Swedish Institute of Computer Science.
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
 *         Contiki shell commands
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "shell-rb.h"
#include "xyzmodem.h"

#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
PROCESS(shell_rb_process, "rb");
SHELL_COMMAND(rb_command,
	      "rb",
	      "rb: ymodem receive",
	      &shell_rb_process);
/*---------------------------------------------------------------------------*/

extern int getc2(void);

// receive ymodem from uart2
PROCESS_THREAD(shell_rb_process, ev, data)
{
    int size;
    int err;
	int res;
	connection_info_t info;
    char ymodemBuf[1024];
  struct process *p;
  PROCESS_BEGIN();

  shell_output_str(&rb_command, "Processes:", "");

  vRbCommand("filename");

#if 0
  size = 0;
  info.mode = xyzModem_ymodem;
  res = xyzModem_stream_open(&info, &err);
  if (!res) {
    while ((res =
	xyzModem_stream_read(ymodemBuf, 1024, &err)) > 0) {
	}
  } else {
    printf("%s\n", xyzModem_error(err));
  }

  xyzModem_stream_close(&err);
  xyzModem_stream_terminate(false, &getc2);
#endif
    
#if 0  
  for(p = PROCESS_LIST(); p != NULL; p = p->next) {
    char namebuf[30];
    strncpy(namebuf, PROCESS_NAME_STRING(p), sizeof(namebuf));
    shell_output_str(&rb_command, namebuf, "");
  }
#endif
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
shell_rb_init(void)
{
  shell_register_command(&rb_command);
}
/*---------------------------------------------------------------------------*/

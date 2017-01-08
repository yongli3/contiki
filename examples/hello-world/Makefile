#APPS = rest-http
CONTIKI_PROJECT = hello-world
CONTIKI_SOURCEFILES += echo-server.c
CFLAGS += -DUIP_CONF_IPV6=0
all: $(CONTIKI_PROJECT)
	arm-none-eabi-objcopy -O ihex hello-world-nosyms.stm32f4discovery flash.hex 
	arm-none-eabi-size hello-world-nosyms.stm32f4discovery

flash:
	ST-LINK_CLI.exe -P flash.hex -V -Rst

UIP_CONF_IPV6=0
CONTIKI = ../..
include $(CONTIKI)/Makefile.include

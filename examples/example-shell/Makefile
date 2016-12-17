CONTIKI_PROJECT = example-shell
all: $(CONTIKI_PROJECT)

PLATFORM_BUILD=1

# serial-shell includes shell app too
APPS = serial-shell
APPS += about 
APPS += telnetd
#APPS += dhcp

CONTIKI = ../..
CONTIKI_WITH_RIME = 1
CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

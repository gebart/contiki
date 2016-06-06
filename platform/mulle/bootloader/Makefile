CONTIKI_PROJECT = bootloader
all: $(CONTIKI_PROJECT)
PROJECT_SOURCEFILES += bootloader_app.c

CFLAGS+=-DPROJECT_CONF_H=\"proj-conf.h\"

CONTIKI=../../..

WITH_UIP6=1
UIP_CONF_IPV6=1

#OPTI=s

CFLAGS+= -DUIP_CONF_IPV6_RPL

PROJECT_SOURCEFILES += coap_reprogram.c bootloader_coap.c 
APPS += rest-engine er-coap

include $(CONTIKI)/Makefile.include

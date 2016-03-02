CONTIKI_PROJECT=border-router
all: $(CONTIKI_PROJECT)

CONTIKI=../../..

#linker optimizations
SMALL=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
PROJECT_SOURCEFILES += slip-bridge.c

#Simple built-in webserver is the default.
#Override with make WITH_WEBSERVER=0 for no webserver.
#WITH_WEBSERVER=webserver-name will use /apps/webserver-name if it can be
#found in the /apps, /platform/$(TARGET)/apps/, or current directory (in that order).
# WITH_WEBSERVER=webserver for /apps/webserver
# WITH_WEBSERVER=raven-webserver for /platform/avr-raven/apps/raven-webserver/
#make clean before changing webservers!

#Note /apps/webserver contains a 2500 byte style sheet which is a severe test
#of the slip connection. Large MSS together with low baud rates without flow
#control will overrun the transmit buffer when the style sheet is requested.

WITH_WEBSERVER=1
ifeq ($(WITH_WEBSERVER),1)
CFLAGS += -DUIP_CONF_TCP=1
CFLAGS += -DWEBSERVER=1
PROJECT_SOURCEFILES += httpd-simple.c
else ifneq ($(WITH_WEBSERVER), 0)
APPS += $(WITH_WEBSERVER)
CFLAGS += -DUIP_CONF_TCP=1
CFLAGS += -DWEBSERVER=2
endif

ifeq ($(PREFIX),)
 PREFIX = fdfd::ff/64
endif

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

BAUDRATE ?= 115200
IFACE_NAME ?= contiki0

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)

connect-router:	$(CONTIKI)/tools/tunslip6
	@#sudo $(CONTIKI)/tools/tunslip6 $(PREFIX)
	@echo 'Starting tunslip6 program in a loop to handle crashes, press CTRL-C twice to exit'
	@echo 'Baud rate is configured to $(BAUDRATE)'
	@echo 'Remember: export BAUDRATE=xxx to run at a different baud rate'
	@echo 'The slip tunnel interface will be named $(IFACE_NAME), export IFACE_NAME=xyz to change this'
	sudo /bin/sh -c 'while true; do "$(CONTIKI)/tools/tunslip6" -B "$(BAUDRATE)" -t "$(IFACE_NAME)" -L -v2 -s \
		"$$($(CONTIKI)/platform/mulle/tools/find-tty.sh $(PROGRAMMER_SERIAL))" "$(PREFIX)"; sleep 1; done'

connect-router-cooja:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -a 127.0.0.1 $(PREFIX)

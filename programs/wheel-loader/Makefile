CONTIKI_PROJECT=wheel-loader
all: $(CONTIKI_PROJECT) 
# use target "er-plugtest-server" explicitly when requried 

CONTIKI=../..

# Contiki IPv6 configuration
WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS += -DUIP_CONF_IPV6=1
CFLAGS += -DUIP_CONF_IPV6_RPL=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

# automatically build RESTful resources
REST_RESOURCES_DIR = ./resources
REST_RESOURCES_FILES = $(notdir $(shell find $(REST_RESOURCES_DIR) -name '*.c'))

PROJECTDIRS += $(REST_RESOURCES_DIR)
PROJECT_SOURCEFILES += $(REST_RESOURCES_FILES)

# linker optimizations
SMALL=1

# REST Engine shall use Erbium CoAP implementation
APPS += er-coap
APPS += rest-engine
APPS += ntpd

include $(CONTIKI)/Makefile.include


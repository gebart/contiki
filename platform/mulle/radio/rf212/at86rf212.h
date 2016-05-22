#ifndef AT86RF212_H
#define AT86RF212_H

#include "contiki.h"

/*============================ MACROS ========================================*/
#define RF212B_VERSION                          (3)
#define RF212_SUPPORTED_MANUFACTURER_ID         (31)

#ifdef RF212_CONF_RX_BUFFERS
#define RF212_RX_BUFFERS RF212_CONF_RX_BUFFERS
#else
#define RF212_RX_BUFFERS 1
#endif

#define CHECKSUM_LEN 2
#define AUX_LEN (CHECKSUM_LEN)

#endif // AT86RF212_H

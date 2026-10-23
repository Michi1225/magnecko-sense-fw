#ifndef ETHERNET_MDNS_H
#define ETHERNET_MDNS_H

#include "nx_api.h"
#include <stdbool.h>

/* Called by the network owner after DHCP/static addressing is reconciled. */
void ethernet_mdns_poll(NX_IP *ip, bool ready, ULONG address, ULONG mask);

#endif

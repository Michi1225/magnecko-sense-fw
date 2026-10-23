#include "ethernet_mdns.h"
#include "app_netxduo.h"
#include "nxd_mdns.h"

static NX_MDNS responder;
/* NetX requires word-aligned cache storage. Keep the worker's stack separate
 * from the small PHY polling thread; all DNS packet parsing runs there. */
static ULONG stack[4096 / sizeof(ULONG)];
static ULONG cache[4096 / sizeof(ULONG)];
static bool created, enabled, service_added, retry_pending;
static ULONG active_address, active_mask, retry_since;

void ethernet_mdns_poll(NX_IP *ip, bool ready, ULONG address, ULONG mask)
{
    if (enabled && (!ready || address != active_address || mask != active_mask)) {
        if (nx_mdns_disable(&responder, 0) != NX_SUCCESS) return;
        enabled = false;
        retry_pending = false;
    }
    if (!ready) return;
    if (enabled && service_added) return;

    /* A failed allocation/join must not turn the 200 ms PHY poll into a busy
     * retry loop. Failure of discovery never prevents IP/TCP operation. */
    ULONG now = tx_time_get();
    if (retry_pending && (ULONG)(now - retry_since) < TX_TIMER_TICKS_PER_SECOND) return;
    retry_pending = true;
    retry_since = now;

    if (!created) {
        if (nx_mdns_create(&responder, ip, nx_get_app_pool(), NX_APP_THREAD_PRIORITY + 1U,
                           stack, sizeof(stack), (UCHAR *)"mgs", cache, sizeof(cache),
                           NX_NULL, 0, NX_NULL) != NX_SUCCESS) return;
        created = true;
    }
    if (!enabled) {
        if (nx_mdns_enable(&responder, 0) != NX_SUCCESS) {
            /* NetX can mark the interface enabled before a failed join or
             * registration. Roll it back before retrying. */
            (void)nx_mdns_disable(&responder, 0);
            return;
        }
        enabled = true;
        active_address = address;
        active_mask = mask;
    }
    if (!service_added) {
        /* Browsers can find the stream endpoint, including its actual host
         * name if another board already owns mgs.local. */
        if (nx_mdns_service_add(&responder, (UCHAR *)"Magnecko Sense",
                                (UCHAR *)"_magnecko._tcp", NX_NULL,
                                (UCHAR *)"version=1", 120, 0, 0, 50002, 1, 0) != NX_SUCCESS) return;
        service_added = true;
    }
    retry_pending = false;
}

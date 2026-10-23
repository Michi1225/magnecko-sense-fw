#include "ethernet_network.h"
#include "ethernet_link.h"
#include "ethernet_mdns.h"
#include "app_events.h"
#include "app_netxduo.h"
#include "main.h"
#include <string.h>
#if MAGNECKO_DHCP
#include "nxd_dhcp_client.h"
static NX_DHCP dhcp;
static bool created, configured, started, fallback, attempted;
static ULONG waiting_since;
static volatile uint32_t dhcp_state, dhcp_error;
static void changed(NX_DHCP *client, UCHAR state) { (void)client; dhcp_state = state; }
#endif
/* A single network owner updates addressing; the transport owns TCP counters.
 * Copy the small telemetry snapshot with interrupts masked, never a NetX call. */
static uint32_t mode, flags, address, mask, gateway, mac_high, mac_low;
static uint32_t link_drops, phy_errors, connections, disconnects, last_reason, last_status, retries, timeouts;
static bool previous_up, previous_phy, client_up;

void ethernet_network_client(bool connected, uint32_t reason, uint32_t status)
{
    if (connected) { if (!client_up) ++connections; }
    else if (client_up) { ++disconnects; last_reason = reason; last_status = status; }
    client_up = connected;
}
void ethernet_network_retry(bool timeout) { if (timeout) ++timeouts; else ++retries; }

void ethernet_network_poll(NX_IP *ip, bool up, bool phy_up, bool phy_error)
{
    if (phy_error) ++phy_errors;
    if (previous_phy && !phy_up) ++link_drops;
    ULONG ip_address = 0, netmask = 0, route = 0;
    uint32_t new_mode = NET_STATIC;
#if MAGNECKO_DHCP
    new_mode = NET_DHCP_WAITING;
    if (!attempted || (!created && up && !previous_up)) {
        attempted = true;
        UINT result = nx_dhcp_create(&dhcp, ip, "magnecko-sense");
        if (result == NX_SUCCESS) {
            created = true;
            result = nx_dhcp_packet_pool_set(&dhcp, nx_get_app_pool());
            if (result == NX_SUCCESS) result = nx_dhcp_state_change_notify(&dhcp, changed);
        }
        dhcp_error = result;
        configured = result == NX_SUCCESS;
    }
    if (!up || !previous_up) {
        if (started) { dhcp_error = nx_dhcp_stop(&dhcp); started = false; }
        if (created && configured) dhcp_error = nx_dhcp_reinitialize(&dhcp);
        (void)nx_ip_address_set(ip, 0, 0);
        (void)nx_ip_gateway_address_set(ip, 0);
        fallback = false;
        waiting_since = tx_time_get();
    }
    if (up && !fallback && !started && configured) {
        dhcp_error = nx_dhcp_start(&dhcp);
        started = dhcp_error == NX_SUCCESS;
    }
    (void)nx_ip_address_get(ip, &ip_address, &netmask);
    bool leased = started && ip_address && netmask &&
        (dhcp_state == NX_DHCP_STATE_BOUND || dhcp_state == NX_DHCP_STATE_RENEWING ||
         dhcp_state == NX_DHCP_STATE_REBINDING);
    if (up && mode == NET_DHCP_BOUND && !leased) waiting_since = tx_time_get();
    if (up && !fallback && !leased && (ULONG)(tx_time_get() - waiting_since) >= 30U * TX_TIMER_TICKS_PER_SECOND) {
        UINT stopped = started ? nx_dhcp_stop(&dhcp) : NX_SUCCESS;
        if (stopped == NX_SUCCESS) started = false;
        if (stopped == NX_SUCCESS) {
            UINT result = nx_ip_address_set(ip, NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK);
            if (result == NX_SUCCESS) {
                (void)nx_ip_gateway_address_set(ip, 0);
                fallback = true;
            } else dhcp_error = result;
        } else dhcp_error = stopped;
    }
    new_mode = fallback ? NET_STATIC_FALLBACK : leased ? NET_DHCP_BOUND : dhcp_error ? NET_ERROR : NET_DHCP_WAITING;
    if (!leased && !fallback) { ip_address = 0; netmask = 0; }
#else
    if (up && !previous_up) (void)nx_ip_address_set(ip, NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK);
#endif
    if (new_mode == NET_STATIC || new_mode == NET_STATIC_FALLBACK)
        (void)nx_ip_address_get(ip, &ip_address, &netmask);
    (void)nx_ip_gateway_address_get(ip, &route);
    ULONG high = 0, low = 0;
    (void)nx_ip_interface_physical_address_get(ip, 0, &high, &low);
    bool ready = up && ip_address && netmask;
    uint32_t next_address = ready ? (uint32_t)ip_address : 0;
    uint32_t next_mask = ready ? (uint32_t)netmask : 0;
    /* Address/lease changes invalidate all TCP services even if they miss the
     * brief IP-ready withdrawal between their own polls. */
    if (address != next_address || mask != next_mask) ethernet_link_invalidate();
    tx_event_flags_set(&app_events, ready ? APP_EVT_ETH_IP_READY : (ULONG)~APP_EVT_ETH_IP_READY, ready ? TX_OR : TX_AND);
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    mode = new_mode; flags = (phy_up ? 1U : 0U) | (up ? 2U : 0U) | (ready ? 4U : 0U);
    address = next_address; mask = next_mask; gateway = ready ? (uint32_t)route : 0;
    mac_high = high; mac_low = low;
    __set_PRIMASK(irq);
    previous_up = up;
    previous_phy = phy_up;
    ethernet_mdns_poll(ip, ready, next_address, next_mask);
}

void ethernet_network_status_packet(uint8_t p[NETWORK_STATUS_SIZE])
{
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    uint32_t values[19] = {HAL_GetTick(), mode, flags | (client_up ? 8U : 0U), address, mask, gateway,
        mac_high, mac_low, ethernet_link_generation(), link_drops, phy_errors, connections, disconnects,
        last_reason, last_status, retries, timeouts,
#if MAGNECKO_DHCP
        dhcp_state, dhcp_error
#else
        0, 0
#endif
    };
    __set_PRIMASK(irq);
    memcpy(p, "NETS", 4); memset(p + 4, 0, 12); p[8] = sizeof(values); p[12] = 1;
    for (unsigned i = 0; i < 19; ++i)
        for (unsigned b = 0; b < 4; ++b) p[16 + 4 * i + b] = (uint8_t)(values[i] >> (8 * b));
}

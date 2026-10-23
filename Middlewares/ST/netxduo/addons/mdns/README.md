# NetX Duo mDNS addon

`nxd_mdns.c` and `nxd_mdns.h` come from the official NetX Duo
[`v6.4.0_rel` tag](https://github.com/eclipse-threadx/netxduo/tree/v6.4.0_rel/addons/mdns),
matching the vendored 6.4.0 core. They use the existing NetX Duo license in
`../../LICENSE.txt`.

Original SHA-256 hashes before the local patch:

- `nxd_mdns.c`: `e2f9c4ea01ade61e6f2a7e5d7636af84c06adbc8c7143243ba1e6e86f9625f0a`
- `nxd_mdns.h`: `aec583db635f8812e23e5f31b669582081b366c0820cb5535cbb399cf5a6daa1`

Local patch: host name conflicts use DNS-compatible `-2`, `-3`, … suffixes;
DNS-SD service instance names retain the upstream ` (2)` format. The upstream
host suffix contains spaces and parentheses, which its own hostname validator
rejects. See `_nx_mdns_conflict_process`.

The application builds the responder only (`NX_MDNS_DISABLE_CLIENT`), advertises
`mgs.local` and `_magnecko._tcp` port 50002, and enables it only with a usable
Ethernet IPv4 address. NetX handles probing, announcements, cache flush,
queries and DHCP address change notifications.

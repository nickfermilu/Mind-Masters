#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// Minimal lwIP config for BLE-only on Pico W
// CYW43 driver compiles WiFi code even when only BLE is used,
// so we need basic ethernet/ARP/IPv4 enabled for compilation.

#define NO_SYS                  1
#define LWIP_SOCKET             0
#define LWIP_NETCONN            0
#define MEM_LIBC_MALLOC         0
#define MEM_ALIGNMENT           4
#define MEM_SIZE                4000
#define MEMP_NUM_TCP_SEG        8
#define LWIP_ARP                1
#define LWIP_ETHERNET           1
#define LWIP_NETIF_HOSTNAME     0
#define LWIP_ICMP               0
#define LWIP_RAW                0
#define LWIP_DHCP               0
#define LWIP_AUTOIP             0
#define LWIP_SNMP               0
#define LWIP_IGMP               0
#define LWIP_DNS                0
#define LWIP_TCP                0
#define LWIP_UDP                0
#define LWIP_IPV4               1
#define LWIP_IPV6               0

#endif

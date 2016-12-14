#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#include "platform-conf.h"

#define arch_isprint(c) ((unsigned)((c) - 0x20) <= (0x7e - 0x20))
#define ctk_arch_isprint arch_isprint

#define ctk_arch_getkey  arch_getkey
#define ctk_arch_keyavail arch_keyavail

#define CH_ULCORNER               '+'
#define CH_URCORNER               '+'
#define CH_LLCORNER               '+'
#define CH_LRCORNER               '+'
#define CH_ENTER                  '\r'
#define CH_DEL                    '\b'
#define CH_CURS_UP                -1
#define CH_CURS_LEFT              -2
#define CH_CURS_RIGHT             -3
#define CH_CURS_DOWN              -4

#define CTK_CONF_MENU_KEY         -5  /* F10 */
#define CTK_CONF_WINDOWSWITCH_KEY -6  /* Ctrl-Tab */
#define CTK_CONF_WIDGETUP_KEY     -7  /* Shift-Tab */
#define CTK_CONF_WIDGETDOWN_KEY   '\t'
#define CTK_CONF_WIDGET_FLAGS     0
#define CTK_CONF_SCREENSAVER      0

#define CTK_CONF_MOUSE_SUPPORT    0
#define CTK_CONF_WINDOWS          1
#define CTK_CONF_WINDOWMOVE       0
#define CTK_CONF_WINDOWCLOSE      0
#define CTK_CONF_ICONS            0
#define CTK_CONF_ICON_BITMAPS     0
#define CTK_CONF_ICON_TEXTMAPS    0
#define CTK_CONF_MENUS            1
#define CTK_CONF_MENUWIDTH        16
#define CTK_CONF_MAXMENUITEMS     10

#define CTK_COLOR_BLACK  (0)
#define CTK_COLOR_BLUE   (1)
#define CTK_COLOR_GRAY   (1 | 2 | 4)
#define CTK_COLOR_CYAN   (1 | 2 | 8)
#define CTK_COLOR_YELLOW (2 | 4 | 8)
#define CTK_COLOR_WHITE  (1 | 2 | 4 | 8)

#define COLOR_BG CTK_COLOR_BLUE

#define BORDERCOLOR         CTK_COLOR_BLACK
#define SCREENCOLOR         CTK_COLOR_BLACK
#define BACKGROUNDCOLOR     CTK_COLOR_BLACK
#define WINDOWCOLOR_FOCUS   CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define WINDOWCOLOR         CTK_COLOR_GRAY   | COLOR_BG * 0x10
#define DIALOGCOLOR         CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define WIDGETCOLOR_HLINK   CTK_COLOR_CYAN   | COLOR_BG * 0x10
#define WIDGETCOLOR_FWIN    CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define WIDGETCOLOR         CTK_COLOR_GRAY   | COLOR_BG * 0x10
#define WIDGETCOLOR_DIALOG  CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define WIDGETCOLOR_FOCUS   CTK_COLOR_YELLOW | COLOR_BG * 0x10
#define MENUCOLOR           CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define OPENMENUCOLOR       CTK_COLOR_WHITE  | COLOR_BG * 0x10
#define ACTIVEMENUITEMCOLOR CTK_COLOR_YELLOW | COLOR_BG * 0x10


#define CCIF
#define CLIF

typedef uint8_t   u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;
typedef unsigned short uip_stats_t;

// Types for clocks and uip_stats
typedef uint32_t clock_time_t;

// rtimer
//typedef uint16_t rtimer_clock_t;
//#define RTIMER_CLOCK_LT(a,b)     ((int16_t)((a)-(b)) < 0)

// The rate of the system clock
#define CLOCK_CONF_SECOND       1000
#define RTIMER_ARCH_SECOND              (60000)


#ifndef   MIN
  #define MIN(a, b)         (((a) < (b)) ? (a) : (b))
#endif

#ifndef   MAX
  #define MAX(a, b)         (((a) > (b)) ? (a) : (b))
#endif

// enable IPV6
#ifndef NETSTACK_CONF_WITH_IPV6
#define  NETSTACK_CONF_WITH_IPV6    1
#endif

#ifndef NETSTACK_CONF_MAC
/* #define NETSTACK_CONF_MAC     csma_driver */
#define NETSTACK_CONF_MAC     nullmac_driver
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
/* #define NETSTACK_CONF_RDC     contikimac_driver */
#define NETSTACK_CONF_RDC     nullrdc_driver
#endif /* NETSTACK_CONF_RDC */

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   nullradio_driver
#endif /* NETSTACK_CONF_RADIO */

#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154
#endif /* NETSTACK_CONF_FRAMER */

#ifndef CC2520_CONF_AUTOACK
#define CC2520_CONF_AUTOACK              1
#endif /* CC2520_CONF_AUTOACK */

#define NULLRDC_CONF_802154_AUTOACK      0

#define WITH_UIP6       1

#if WITH_UIP6
/* Network setup for IPv6 */
#define NETSTACK_CONF_NETWORK sicslowpan_driver

/* Specify a minimum packet size for 6lowpan compression to be
   enabled. This is needed for ContikiMAC, which needs packets to be
   larger than a specified size, if no ContikiMAC header should be
   used. */
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63
#define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER 0

#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                8
#endif

#else /* WITH_UIP6 */

/* Network setup for non-IPv6 (rime). */

#define NETSTACK_CONF_NETWORK rime_driver

#define COLLECT_CONF_ANNOUNCEMENTS       1
#define CXMAC_CONF_ANNOUNCEMENTS         0
#define XMAC_CONF_ANNOUNCEMENTS          0
#define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

#define CONTIKIMAC_CONF_COMPOWER         1
#define XMAC_CONF_COMPOWER               1
#define CXMAC_CONF_COMPOWER              1

#ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
#define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     32
#endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM                16
#endif /* QUEUEBUF_CONF_NUM */

#ifndef TIMESYNCH_CONF_ENABLED
#define TIMESYNCH_CONF_ENABLED           0
#endif /* TIMESYNCH_CONF_ENABLED */

#if TIMESYNCH_CONF_ENABLED
/* CC2520 SDF timestamps must be on if timesynch is enabled. */
#undef CC2520_CONF_SFD_TIMESTAMPS
#define CC2520_CONF_SFD_TIMESTAMPS       1
#endif /* TIMESYNCH_CONF_ENABLED */

#endif /* WITH_UIP6 */

#define PACKETBUF_CONF_ATTRS_INLINE 1

#ifndef RF_CHANNEL
#define RF_CHANNEL              11
#endif /* RF_CHANNEL */

#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT 0

#define IEEE802154_CONF_PANID       0x0777

#define SHELL_VARS_CONF_RAM_BEGIN 0x1100
#define SHELL_VARS_CONF_RAM_END 0x2000

#define PROFILE_CONF_ON 0
#define ENERGEST_CONF_ON 0

#define ELFLOADER_CONF_TEXT_IN_ROM 0
#ifndef ELFLOADER_CONF_DATAMEMORY_SIZE
#define ELFLOADER_CONF_DATAMEMORY_SIZE 0x400
#endif /* ELFLOADER_CONF_DATAMEMORY_SIZE */
#ifndef ELFLOADER_CONF_TEXTMEMORY_SIZE
#define ELFLOADER_CONF_TEXTMEMORY_SIZE 0x800
#endif /* ELFLOADER_CONF_TEXTMEMORY_SIZE */


#define AODV_COMPLIANCE
#define AODV_NUM_RT_ENTRIES 32

#define WITH_ASCII 1

#define PROCESS_CONF_NUMEVENTS 8
#define PROCESS_CONF_STATS 1
/*#define PROCESS_CONF_FASTPOLL    4*/

#ifdef WITH_UIP6

#define UIP_CONF_ICMP6                  1
#define LINKADDR_CONF_SIZE              8

#define UIP_CONF_LL_802154              1
#define UIP_CONF_LLH_LEN                0

#define UIP_CONF_ROUTER                 0
#ifndef UIP_CONF_IPV6_RPL
#define UIP_CONF_IPV6_RPL               0
#endif /* UIP_CONF_IPV6_RPL */

/* configure number of neighbors and routes */
#ifndef UIP_CONF_DS6_NBR_NBU
#define UIP_CONF_DS6_NBR_NBU     30
#endif /* UIP_CONF_DS6_NBR_NBU */
#ifndef UIP_CONF_DS6_ROUTE_NBU
#define UIP_CONF_DS6_ROUTE_NBU   30
#endif /* UIP_CONF_DS6_ROUTE_NBU */

#define UIP_CONF_ND6_SEND_NA		1
#define UIP_CONF_ND6_SEND_RA		0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IPV6                   1
#ifndef UIP_CONF_IPV6_QUEUE_PKT
#define UIP_CONF_IPV6_QUEUE_PKT         0
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      4
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0
#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE		1000
#endif

#define SICSLOWPAN_CONF_COMPRESSION_IPV6        0
#define SICSLOWPAN_CONF_COMPRESSION_HC1         1
#define SICSLOWPAN_CONF_COMPRESSION_HC01        2
#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#ifndef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
#endif /* SICSLOWPAN_CONF_FRAG */
#define SICSLOWPAN_CONF_CONVENTIONAL_MAC	1
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2
#ifndef SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS   5
#endif /* SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS */
#else /* WITH_UIP6 */
#define UIP_CONF_IP_FORWARD      1
#define UIP_CONF_BUFFER_SIZE     1000
#endif /* WITH_UIP6 */

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LLH_LEN         0
#ifndef  UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW  48
#endif
#ifndef  UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS         200
#endif
#define UIP_CONF_MAX_CONNECTIONS 4
#define UIP_CONF_MAX_LISTENPORTS 8
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
//#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         1

#define UIP_CONF_TCP            1
#define UIP_CONF_TCP_SPLIT       0

#define LOG_CONF_ENABLED         1


/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#endif /* CONTIKI_CONF_H */

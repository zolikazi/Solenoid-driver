/*
 * TODO copyright header
 */

#ifndef __LWIP_PACKET_H__
#define __LWIP_PACKET_H__

#include "lwip/opt.h"

#if LWIP_PACKET /* don't build if not configured for use in lwipopts.h */

#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "netif/etharp.h"


#ifdef __cplusplus
extern "C" {
#endif

struct packet_pcb;

/**
 * Function prototype for packet pcb receive callback functions.
 * @param arg user supplied argument (packet_pcb.recv_arg)
 * @param pcb the packet_pcb which received data
 * @param p the packet buffer that was received
 * @param addr the remote Ethernet address from which the packet was received
 * @return 1 if the packet was 'eaten' (aka. deleted),
 *         0 if the packet lives on
 * If returning 1, the callback is responsible for freeing the pbuf
 * if it's not used any more.
 */
typedef u8_t (*packet_recv_fn)(void *arg, struct packet_pcb *pcb, struct pbuf *p, struct eth_addr *addr);

struct packet_pcb {
  /** Linked list */
  struct packet_pcb *next;
  /** Ethertype associated with this PCB in network byte order */
  u16_t protocol;
  /** Network interface to send packets on */
  struct netif *netif;
  /** receive callback function */
  packet_recv_fn recv;
  /** user-supplied argument for the recv callback */
  void *recv_arg;
};

/* Application layer interface */
struct packet_pcb * packet_new      (u16_t proto);
void                packet_remove   (struct packet_pcb *pcb);
err_t               packet_bind     (struct packet_pcb *pcb, struct netif *netif);

void                packet_recv     (struct packet_pcb *pcb, packet_recv_fn recv, void *recv_arg);
err_t               packet_sendto   (struct packet_pcb *pcb, struct pbuf *p, struct eth_addr *addr);

/* Lower layer interface */
u8_t                packet_input    (struct pbuf *p, struct netif *netif);

#ifdef __cplusplus
}
#endif


#endif /* LWIP_PACKET */

#endif /* __LWIP_PACKET_H__ */

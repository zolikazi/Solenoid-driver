/**
 * @file
 * Implementation of packet protocol PCBs for low-level handling of
 * different types of protocols besides (or overriding) those
 * already available in lwIP.
 *
 */

/*
 * TODO copyright header
 */

#include "lwip/opt.h"

#if LWIP_PACKET /* don't build if not configured for use in lwipopts.h */

#include "lwip/memp.h"
#include "lwip/packet.h"

#include <string.h> /* for memset() */

/** The list of Packet PCBs */
static struct packet_pcb *packet_pcbs;

/**
 * Create a Packet PCB.
 *
 * @return The Packet PCB which was created. NULL if the PCB data structure
 * could not be allocated.
 *
 * @param proto the protocol number of the Ethernet payload (e.g. ETHTYPE_PTP) in host byte order.
 *
 * @see packet_remove()
 */
struct packet_pcb *
packet_new(u16_t proto)
{
  struct packet_pcb *pcb = NULL;

  LWIP_DEBUGF(PACKET_DEBUG | LWIP_DBG_TRACE, ("packet_new\n"));

  pcb = (struct packet_pcb *)memp_malloc(MEMP_PACKET_PCB);
  if (pcb) {
    memset(pcb, 0, sizeof(struct packet_pcb));
    pcb->protocol = PP_HTONS(proto);
    pcb->next = packet_pcbs;
    packet_pcbs = pcb;
  }

  return pcb;
}

/**
 * Remove a Packet PCB.
 *
 * @param pcb Packet PCB to be removed. The PCB is removed from the list of
 * Packet PCBs and the data structure is freed from memory.
 *
 * @see packet_new()
 */
void
packet_remove(struct packet_pcb *pcb)
{
  struct packet_pcb *p;

  if (packet_pcbs == pcb) {
    packet_pcbs = packet_pcbs->next;
  } else {
    for (p=packet_pcbs; p; p=p->next) {
      if (p->next == pcb) {
        p->next = pcb->next;
      }
    }
  }
  memp_free(MEMP_PACKET_PCB, pcb);
}

/**
 * Bind a Packet PCB to a network interface.
 *
 * A Packet PCB that is bound to a netif sends packets on that netif
 * and receives packets only on that netif.
 * Unbound Packet PCBs send on the default netif.
 * To unbind a bound Packet PCBs, use packet_bind(pcb, NULL).
 *
 * @param pcb Packet PCB to be bound.
 *
 * @return lwIP error code.
 * - ERR_OK. Successful. No error occured.
 * - ERR_CONN. TODO check if the specified netif is valid? how?
 */
err_t
packet_bind(struct packet_pcb *pcb, struct netif *netif)
{
  pcb->netif = netif;

  return ERR_OK;
}

/**
 * Set the callback function for received packets that match the
 * Packet PCB's protocol and netif.
 *
 * The callback function MUST either
 * - eat the packet by calling pbuf_free() and returning non-zero. The
 *   packet will not be passed to other packet PCBs or other protocol layers.
 * - not free the packet, and return zero. The packet will be matched
 *   against further PCBs and/or forwarded to another protocol layers.
 *
 * @return non-zero if the packet was free()d, zero if the packet remains
 * available for others.
 *
 */
void
packet_recv(struct packet_pcb *pcb, packet_recv_fn recv, void *recv_arg)
{
  pcb->recv = recv;
  pcb->recv_arg = recv_arg;
}

/**
 * Package the given data into an Ethernet header and send it to the specified
 * address. Uses pcb->protocol for EtherType.
 *
 * @param pcb Packet PCB on which to send.
 * @param p the payload to send.
 * @param addr the destination of the packet.
 *
 * @return ERR_OK if no error occurred.
 */
err_t
packet_sendto(struct packet_pcb *pcb, struct pbuf *p, struct eth_addr *addr)
{
  struct netif *netif = pcb->netif ? pcb->netif : netif_default;
  struct eth_hdr *ethhdr;

  LWIP_ASSERT("netif != NULL", netif != NULL);
  LWIP_ASSERT("p != NULL", p != NULL);
  LWIP_ASSERT("addr != NULL", addr != NULL);

  if (p->len > netif->mtu) {
    //TODO LWIP_DEBUGF
    return ERR_BUF;
  }

  /* add Ethernet header */
  if (pbuf_header(p, sizeof(struct eth_hdr)) != 0) {
    //TODO LWIP_DEBUGF
    return ERR_BUF;
  }

  /* Fill in the fields */
  ethhdr = (struct eth_hdr *)p->payload;
  ETHADDR32_COPY(&ethhdr->dest, addr);
  ETHADDR16_COPY(&ethhdr->src, netif->hwaddr);
  ethhdr->type = pcb->protocol;

  return netif->linkoutput(netif, p);
}

/**
 * Determine if in incoming packet is covered by a Packet PCB
 * and if so, pass it to a user-provided receive callback function.
 *
 * @param p pbuf of the incoming packet
 * @param netif the netif that received the packet
 *
 * @return 1 if the packet was eaten by a PCB, 0 if the packet lives on
 */
u8_t
packet_input(struct pbuf *p, struct netif *netif)
{

  u8_t eaten = 0;
  struct packet_pcb *pcb = packet_pcbs;

  s16_t header_len = SIZEOF_ETH_HDR;
  struct eth_hdr* ethhdr = (struct eth_hdr *)p->payload;
  u16_t type = ethhdr->type;

#if ETHARP_SUPPORT_VLAN
  /* jump over VLAN tags, and set header length */
  while (type == PP_HTONS(ETHTYPE_VLAN)) {
    struct eth_vlan_hdr *vlan = (struct eth_vlan_hdr*)(p->payload+header_len);

    if (p->len <= header_len + SIZEOF_VLAN_HDR) {
      /* there is nothing after the VLAN header */
      /* note that this has already been checked for the first VLAN tag in ethernet_input() */
      return 0;
    }

    header_len += SIZEOF_VLAN_HDR;
    type = vlan->tpid;
  }
#endif /* ETHARP_SUPPORT_VLAN */

  /* hide Ethernet header from the application */
  if (pbuf_header(p, -header_len)) {
    /* something is very wrong, bail out completely */
    pbuf_free(p);
    return 1;
  }

  /* look for a matching PCB */
  while (!eaten && pcb) {
    if (pcb->protocol == type && pcb->recv) {
      if (pcb->netif && pcb->netif != netif) {
      } else {
        eaten = pcb->recv(pcb->recv_arg, pcb, p, &ethhdr->src);
      }
    }
    pcb = pcb->next;
  }

  if (!eaten) {
    /* unhide Ethernet header */
    if (pbuf_header(p, header_len)) {
      /* something is very wrong, bail out completely */
      pbuf_free(p);
      return 1;
    }
  }

  return eaten;
}


#endif /* LWIP_PACKET */

/**
  ******************************************************************************
  * @file    ptpsync.c
  * @author  Miklos Mate
  * @version V1.0.0
  * @brief   PTP Sync message sending on STM32
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

#define ETHARP_SUPPORT_VLAN	1

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "netif/etharp.h"
#include <string.h>
#include <stdio.h>
//#include "spi_pov.h"
#include "stm32f4x7_eth.h"

/* Private typedef -----------------------------------------------------------*/

PACK_STRUCT_BEGIN
struct ptpsync_msg {
    // header
    PACK_STRUCT_FIELD(uint8_t  transport_type);
    PACK_STRUCT_FIELD(uint8_t  version);
    PACK_STRUCT_FIELD(uint16_t length);
    PACK_STRUCT_FIELD(uint8_t  domain);
    PACK_STRUCT_FIELD(uint8_t  reserved1);
    PACK_STRUCT_FIELD(uint16_t flags);
    PACK_STRUCT_FIELD(uint32_t correction_msb);
    PACK_STRUCT_FIELD(uint32_t correction_lsb);
    PACK_STRUCT_FIELD(uint32_t reserved2);
    PACK_STRUCT_FIELD(uint8_t  clockid[8]);
    PACK_STRUCT_FIELD(uint16_t srcport);
    PACK_STRUCT_FIELD(uint16_t seq);
    PACK_STRUCT_FIELD(uint8_t  control);
    PACK_STRUCT_FIELD(uint8_t  logmsgperiod);
    // sync
    PACK_STRUCT_FIELD(uint16_t ts_sec_msb);
    PACK_STRUCT_FIELD(uint32_t ts_sec_lsb);
    PACK_STRUCT_FIELD(uint32_t ts_nsec);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Packet structure and header pointers---------------------------------------*/
static u8_t   data[1500];
static struct eth_hdr *eth;
static struct eth_vlan_hdr *vlan;
static struct ip_hdr *iph;
static struct udp_hdr *udph;
static struct ptpsync_msg *syncmsg;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void ptp_sync_init(){

  /* Initialize packet header structure  */
  eth = (struct eth_hdr *) data;
  vlan = (struct eth_vlan_hdr*)(data + sizeof(struct eth_hdr));
  iph = (struct ip_hdr *) ((uint8_t*)vlan + sizeof(struct eth_vlan_hdr));
  udph = (struct udp_hdr *) ((uint8_t*)iph + sizeof(struct ip_hdr));
  //phdr = (struct pov_hdr *) ((uint8_t*)udph + sizeof(struct udp_hdr));
  //payload = (char *)((uint8_t*)phdr + sizeof(struct pov_hdr));
  syncmsg = (struct ptpsync_msg *) ((uint8_t*)udph + sizeof(struct udp_hdr));

  /* Construct the Ethernet header */

  /* Ethernet header */
  eth->src.addr[0] = MAC_ADDR0;
  eth->src.addr[1] = MAC_ADDR1;
  eth->src.addr[2] = MAC_ADDR2;
  eth->src.addr[3] = MAC_ADDR3;
  eth->src.addr[4] = MAC_ADDR4;
  eth->src.addr[5] = MAC_ADDR5;

  eth->dest.addr[0] = DEST_MAC_ADDR0;
  eth->dest.addr[1] = DEST_MAC_ADDR1;
  eth->dest.addr[2] = DEST_MAC_ADDR2;
  eth->dest.addr[3] = DEST_MAC_ADDR3;
  eth->dest.addr[4] = DEST_MAC_ADDR4;
  eth->dest.addr[5] = DEST_MAC_ADDR5;

  /* Ethertype field */
//	eth->type = htons(0x800);
  eth->type = htons(0x8100);

  /* VLAN header */
  vlan->prio_vid = htons(0xC041);
  vlan->tpid = htons(0x800);

  //Fill in the IP Header
  IPH_VHL_SET(iph, 4, IP_HLEN/4);
  IPH_TOS_SET(iph, 0x18);
  //IPH_LEN_SET(iph, htons(sizeof (struct ip_hdr) + sizeof (struct udp_hdr) + payload_len));
  IPH_ID_SET(iph, 1);
  IPH_OFFSET_SET(iph, 0);
  IPH_TTL_SET(iph, 32);
  IPH_PROTO_SET(iph, IP_PROTO_UDP);
  IPH_CHKSUM_SET(iph, 0);
  IP4_ADDR(&iph->src,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP4_ADDR(&iph->dest,DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3);

  //Fill in the UDP Header
  udph->src = htons(319);
  udph->dest = htons(319);
//  udph->len = htons(8 + payload_len);
  udph->chksum = 0;

  //Fill in the PTP Header
  syncmsg->transport_type = 0;
  syncmsg->version = 2;
  syncmsg->length = htons(44);
  syncmsg->domain = 0;
  syncmsg->reserved1 = 0;
  syncmsg->flags = htons(8); //PTP_TIMESCALE
  syncmsg->correction_msb = htonl(0);
  syncmsg->correction_lsb = htonl(0);
  syncmsg->reserved2 = 0;
  memcpy(syncmsg->clockid, "NEMTOMMI", 8);
  syncmsg->srcport = htons(3);
  syncmsg->seq = 0;
  syncmsg->control = 0;
  syncmsg->logmsgperiod = 0;
}

void ptp_sync_send()
{
  struct pbuf *p;
  int tx_len = sizeof(struct eth_hdr)+sizeof(struct eth_vlan_hdr)+sizeof(struct ip_hdr)+sizeof(struct udp_hdr)+sizeof(struct ptpsync_msg);
  err_t err;
  static uint16_t seq = 0;

  // Update payload dependent header fields
  IPH_LEN_SET(iph, htons(sizeof (struct ip_hdr) + sizeof (struct udp_hdr) + sizeof (struct ptpsync_msg)));
  udph->len = htons(8 + sizeof (struct ptpsync_msg));

  syncmsg->seq = htons(seq);
  syncmsg->ts_sec_msb = 0;
  syncmsg->ts_sec_lsb = htonl(ETH_GetPTPRegister(ETH_PTPTSHR));
  syncmsg->ts_nsec = htonl(ETH_PTPSubSecond2NanoSecond(ETH_GetPTPRegister(ETH_PTPTSLR)));


  p = pbuf_alloc(PBUF_TRANSPORT, tx_len, PBUF_POOL);
  if (p != NULL) {
    /* copy data to pbuf */
    pbuf_take(p, (char*)data, tx_len);

    /* send udp data */
    err = netif_default->linkoutput(netif_default, p);
    //tfp_printf("sent %d, %d\n", tx_len, err);

    /* free pbuf */
    pbuf_free(p);
  }
  seq++;
  return ;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

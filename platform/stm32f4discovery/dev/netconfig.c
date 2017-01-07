#include "main.h"
#include <stm32f4x7_eth.h>
#include <stm32f4x7_eth_bsp.h>
#include <debug-uart.h>

#include <stdio.h>
#include <string.h>

extern ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];
extern ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];

extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE];
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE];

extern ETH_DMADESCTypeDef *DMATxDescToSet;

extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;

void uip_log(char *msg)
{
	printf(msg);
}

void net_init()
{
	int i;
  uint8_t mac_addr[6];

	mac_addr[0] = MAC_ADDR0;
	mac_addr[1] = MAC_ADDR1;
	mac_addr[2] = MAC_ADDR2;
	mac_addr[3] = MAC_ADDR3;
	mac_addr[4] = MAC_ADDR4;
	mac_addr[5] = MAC_ADDR5;

	ETH_BSP_Config();

	ETH_MACAddressConfig(ETH_MAC_Address0, mac_addr);

	ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
	ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

	for (i = 0; i < ETH_TXBUFNB; i++)
	{
		ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumByPass);
	}

	ETH_Start();
}

uint16_t net_receive(uint8_t *uipbuf)
{
	uint32_t i;
	FrameTypeDef frame;
	__IO ETH_DMADESCTypeDef *DMARxNextDesc;
	uint16_t len = 0;

	if (ETH_CheckFrameReceived())
	{
		frame = ETH_Get_Received_Frame();
		len = frame.length;

		//printf("Frame length: %d\n", len);
		
		// FIXME is it ok?
		memcpy(uipbuf, (uint8_t *)frame.buffer, len);
		/*
		for (i = 0; i < len; i++)
		{
						printf("%x ", uipbuf[i]);
		}
		printf("\n");

		printf("Copied to uipbuf\n");
		*/
		
		/* Release descriptors to DMA */
		/* Check if frame with multiple DMA buffer segments */
		if (DMA_RX_FRAME_infos->Seg_Count > 1)
		{
			DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
		}
		else
		{
			DMARxNextDesc = frame.descriptor;
		}

		/* Set Own bit in Rx descriptors: givrs the buffers back to DMA */
		for (i = 0; i < DMA_RX_FRAME_infos->Seg_Count; i++)
		{
			DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
			DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
		}

		/* Clear Segment_Count */
		DMA_RX_FRAME_infos->Seg_Count = 0;

		/* When Rx Buffer unavailable flag is set: clear it and resume reception */
		if ((ETH->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
		{
			/* Clear RBUS ETHERNET DMA flag */
			ETH->DMASR = ETH_DMASR_RBUS;
			/* Resume DMA Reception */
			ETH->DMARPDR = 0;
		}
	}

	return len;
}

void net_send(uint8_t *uip_buf, uint16_t uip_len)
{
	memcpy((uint8_t *)DMATxDescToSet->Buffer1Addr, uip_buf, uip_len);

	ETH_Prepare_Transmit_Descriptors(uip_len);
}

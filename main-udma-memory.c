/*
 * main-udma-memory.c
 *
 *  Created on: 24 de nov de 2022
 *      Author: rogerio.marcondeli
 */

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_uart.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>
#include <driverlib/udma.h>

volatile uint32_t g_ui32SysClock;

uint8_t DMA_Control_Table[1024];

uint8_t Source_Buffer[256];

uint8_t Dest_Buffer[256];

uint8_t Source_Buffer[256]="uDMA Test";


int main(void) {

	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000); //Set CLK to 120Mhz


	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA))
	{
	}

	uDMAEnable();

	uDMAControlBaseSet(&DMA_Control_Table);

	uDMAChannelAttributeDisable(UDMA_CHANNEL_SW, UDMA_ATTR_ALL);

	uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
						  UDMA_SIZE_8 | UDMA_SRC_INC_8 |
						  UDMA_DST_INC_8 | UDMA_ARB_8);

	uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,
						   UDMA_MODE_AUTO, Source_Buffer, Dest_Buffer, sizeof(Dest_Buffer));

	uDMAChannelEnable(UDMA_CHANNEL_SW);
	uDMAChannelRequest(UDMA_CHANNEL_SW);

	while(1)
	{

	}
	return 0;
}


/*
 * main-adc-udma.c
 *
 *  Created on: 23 de nov de 2022
 *      Author: rogerio.marcondeli
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "inc/hw_emac.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/emac.h"
#include "driverlib/rom.h"

#define ADC_SAMPLE_BUF_SIZE (8)

volatile uint32_t g_ui32SysClock;

static uint32_t g_ui32DMAErrCount = 0;

uint32_t udmaCtrlTable[1024/sizeof(uint32_t)]__attribute__((aligned(1024)));

uint32_t ADC_OUT[8];

uint32_t n=0;

float Gain = 0.00439453125; // 18V/4096
float Value = 0.0;
static int Adc_Value = 0;

void init_ADC();

void uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	ui32Status = uDMAErrorStatusGet();

	if(ui32Status)
	{
		uDMAErrorStatusClear();
		g_ui32DMAErrCount++;
	}

}

void ADCprocess(uint32_t ch)
{
	if ((((tDMAControlTable *) udmaCtrlTable)[ch].ui32Control & UDMA_CHCTL_XFERMODE_M) != UDMA_MODE_STOP) return;

	uDMAChannelTransferSet(ch, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_OUT, ADC_SAMPLE_BUF_SIZE);

	uDMAChannelEnable(UDMA_CHANNEL_ADC0);
}

void ADCseq0Handler()
{
	ADCIntClear(ADC0_BASE, 0);
	n++;
	ADCprocess(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
	ADCprocess(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
}

int main(void)
{

	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000); //Set CLK to 120Mhz

	SysCtlDelay(20);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);    //Enable the clock to ADC module
	SysCtlDelay(10); // Time for the peripheral enable to set

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	SysCtlDelay(10);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	SysCtlDelay(30);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);
	SysCtlDelay(80);

	IntMasterEnable();

	init_ADC();

	while(1)
	{
		Adc_Value = ADC_OUT[3];
		Value = (float)Adc_Value * Gain;
	}
}

void init_ADC()
{
	//ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);

	// ADCClockConfigSet(uint32_t ui32Base, uint32_t ui32Config, uint32_t ui32ClockDiv)
	// ui32Base is the base address of the ADC configure
	// ui32Config is the value used to configure the ADC clock input. For TM4C123 device
	// ADC_CLOCK_SRC_PIOSC is PLL/25 always thus 400MHz/25 = 16 MHz
	// ui32ClockDiv is the input clock divider which is in the range of 1 to 64

	ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

	ADCHardwareOversampleConfigure(ADC0_BASE, 64);

	// Oversampling average multiple samples from same analog input. Rates suported are
	// 2x, 4x, 8x, 16x, 32x, 64x. If set to 0 hardware oversampling is disabled

	SysCtlDelay(10); // Time for the clock configuration to set

	IntDisable(INT_ADC0SS0);
	ADCIntDisable(ADC0_BASE, 0);
	ADCSequenceDisable(ADC0_BASE,0);

	// With sequence disabled, it is now safe to load the new configuration parameters

	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);

	// ADCSequenceConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Trigger, uint32_t ui32Priority)
	// ui32Base is the base address of the ADC configure and must always be ADC0_BASE
	// ui32SequenceNum is the sample sequence number (depends on sequencer used)
	// ui32Trigger is the trigger source that innitiates the sampling sequence, in this case the ADC is always running
	// ui32Priority is the relative priority with respect to other sample sequences 0 being the highest priority and 3 being the lowest


	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH4); // VOLTAGE_2
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH5); // VOLTAGE_1
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH6); // VOLTAGE_3
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH7); // VOLTAGE_4
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH12); // CURRENT_4
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH13); // LV_2X_SIGNAL1
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH14); // LV_2X_SIGNAL2
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH16 | ADC_CTL_END | ADC_CTL_IE); // DRIVER_VOLT

	// ADCSequenceStepConfigure(uint32_t ui32Base, uint32_t ui32SequenceNum, uint32_t ui32Step, uint32_t ui32Config)
	// ui32Base is the base address of the ADC configure and must always be ADC0_BASE
	// ui32SequenceNum is the sample sequence number (depends on sequencer used)
	// ui32Step is the step to be configured
	// ui32Config is the configuration of this step. Routs it to an analog input. Also defines the last in the sequence with ADC_CTL_END and also fires an interrupt every 8 samples with ADC_CTL_IE

	ADCSequenceEnable(ADC0_BASE,0); //Once configuration is set, re-enable the sequencer
	ADCIntClear(ADC0_BASE,0);
	uDMAEnable(); // Enables uDMA
	uDMAControlBaseSet(udmaCtrlTable);

	// Configures the base address of the channel control table. Table resides in system memory and holds control
	// information for each uDMA channel. Table must be aligned on a 1024-byte boundary. Base address must be
	// configured before any of the channel functions can be used

	ADCSequenceDMAEnable(ADC0_BASE,0);
	// Allows DMA requests to be generated based on the FIFO level of the sample sequencer (SS0)

	uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	// Start with Ping-pong PRI side, low priority, unmask

	uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);
	// Only allow burst transfers

	uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_128);
	uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_128);

	// uDMAChannelControlSet(uint32_t ui32ChannelStructIndex, uint32_t ui32Control)
	// ui32Control is the logical OR of five values: the data size, the source address incremen, the destination address
	// increment, the arbitration size and the burst flag

	uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_OUT, ADC_SAMPLE_BUF_SIZE);
	uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_OUT, ADC_SAMPLE_BUF_SIZE);

	// uDMAChannelTransferSet(uint32_t ui32ChannelStructIndex, uint32_t ui32Mode, void *pvSrcAddr, void *pvDstAddr, uint32_t ui32TransferSize)
	// pvSrcAddr is the source address for the transfer in this case from hw_adc.h ADC_O_SSFIFO0 is the result of the FIFO 0 register
	// pvDstAddr is the destination address for the transfer
	// ui32TransferSize is the number of data items to transfer

	ADCIntEnable(ADC0_BASE,0);
	IntEnable(INT_ADC0SS0);

	//ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0); // Enables ADC interrupt source due to DMA on ADC sample sequence 0

	uDMAChannelEnable(UDMA_CHANNEL_ADC0); // Enables DMA channel so it can perform transfers
}

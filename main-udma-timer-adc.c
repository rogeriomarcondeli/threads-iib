/*
 * main-udma-timer-adc.c
 *
 *  Created on: 25 de nov de 2022
 *      Author: rogerio.marcondeli
 */

//*****************************************************************************

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
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/systick.h"

//*****************************************************************************

typedef struct
{
    float Value;
    int Adc_Value;
}adc_t;

//*****************************************************************************

#define ADC_SAMPLE_BUF_SIZE         7

//*****************************************************************************

static uint8_t ControlTable[1024] __attribute__ ((aligned(1024)));

//*****************************************************************************

enum BUFFER_STATUS_ADC0
{
	EMPTY_ADC_0,
	FILLING_ADC_0,
	FULL_ADC_0
};

//*****************************************************************************

enum BUFFER_STATUS_ADC1
{
	EMPTY_ADC_1,
	FILLING_ADC_1,
	FULL_ADC_1
};

//*****************************************************************************

uint16_t ADC0Buffer1[ADC_SAMPLE_BUF_SIZE];
uint16_t ADC0Buffer2[ADC_SAMPLE_BUF_SIZE];

//*****************************************************************************

uint16_t ADC1Buffer1[ADC_SAMPLE_BUF_SIZE];
uint16_t ADC1Buffer2[ADC_SAMPLE_BUF_SIZE];

//*****************************************************************************

static enum BUFFER_STATUS_ADC0 BufferStatusADC0[2];

//*****************************************************************************

static enum BUFFER_STATUS_ADC1 BufferStatusADC1[2];

//*****************************************************************************

uint32_t adc0_int_count = 0;

//*****************************************************************************

uint32_t adc1_int_count = 0;

//*****************************************************************************

volatile uint32_t g_ui32SysClock;

//*****************************************************************************

extern adc_t VoltageCh1;
extern adc_t VoltageCh2;
extern adc_t VoltageCh3;
extern adc_t VoltageCh4;

extern adc_t CurrentCh1;
extern adc_t CurrentCh2;
extern adc_t CurrentCh3;
extern adc_t CurrentCh4;

extern adc_t LvCurrentCh1;
extern adc_t LvCurrentCh2;
extern adc_t LvCurrentCh3;

extern adc_t DriverVolt;
extern adc_t Driver1Curr;
extern adc_t Driver2Curr;

//*****************************************************************************

adc_t VoltageCh1;
adc_t VoltageCh2;
adc_t VoltageCh3;
adc_t VoltageCh4;

adc_t CurrentCh1;
adc_t CurrentCh2;
adc_t CurrentCh3;
adc_t CurrentCh4;

adc_t LvCurrentCh1;
adc_t LvCurrentCh2;
adc_t LvCurrentCh3;

adc_t DriverVolt;
adc_t Driver1Curr;
adc_t Driver2Curr;

//*****************************************************************************

float CurrentRange(float nFstCurr, float nSecCurr, float nBurden, float MaxVoltInput)
{
    float Ix, Xv = 0.0;

    Xv = nSecCurr*nBurden;
    Ix = nFstCurr*MaxVoltInput;
    Ix = Ix/Xv;

    return Ix;
}

//*****************************************************************************

void InitADCs(void);
float CurrentRange(float nFstCurr, float nSecCurr, float nBurden, float MaxVoltInput);

//*****************************************************************************

void uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	ui32Status = uDMAErrorStatusGet();

	if(ui32Status)
	{
		uDMAErrorStatusClear();

	}
}

//*****************************************************************************

void ADC0SS0_Handler(void)
{
	adc0_int_count++;

	HWREG(ADC0_BASE + ADC_O_ISC) = HWREG(ADC0_BASE + ADC_O_RIS) & (1 << 8);

	if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) ==
			                UDMA_MODE_STOP) && (BufferStatusADC0[0] == FILLING_ADC_0)) {
		BufferStatusADC0[0] = FULL_ADC_0;
		BufferStatusADC0[1] = FILLING_ADC_0;
	} else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) ==
                                 UDMA_MODE_STOP) && (BufferStatusADC0[1] == FILLING_ADC_0)) {
		BufferStatusADC0[0] = FILLING_ADC_0;
	    BufferStatusADC0[1] = FULL_ADC_0;
	}

	if(BufferStatusADC0[0] == FULL_ADC_0) {
		BufferStatusADC0[0] = EMPTY_ADC_0;

		uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
		                       UDMA_MODE_PINGPONG,
		                       (void *)(ADC0_BASE + ADC_O_SSFIFO0),
		                       ADC0Buffer1, ADC_SAMPLE_BUF_SIZE);

		uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
	} else if(BufferStatusADC0[1] == FULL_ADC_0) {

		BufferStatusADC0[1] = EMPTY_ADC_0;

		uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
		 		               UDMA_MODE_PINGPONG,
		 		               (void *)(ADC0_BASE + ADC_O_SSFIFO0),
		 		               ADC0Buffer2, ADC_SAMPLE_BUF_SIZE);

		uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
	}

}

//*****************************************************************************

void ADC1SS0_Handler(void)
{
	adc1_int_count++;

	HWREG(ADC1_BASE + ADC_O_ISC) = HWREG(ADC1_BASE + ADC_O_RIS) & (1 << 8);

	if ((uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) ==
			                UDMA_MODE_STOP) && (BufferStatusADC1[0] == FILLING_ADC_1)) {
		BufferStatusADC1[0] = FULL_ADC_1;
		BufferStatusADC1[1] = FILLING_ADC_1;
	} else if ((uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) ==
                                 UDMA_MODE_STOP) && (BufferStatusADC1[1] == FILLING_ADC_1)) {
		BufferStatusADC1[0] = FILLING_ADC_1;
	    BufferStatusADC1[1] = FULL_ADC_1;
	}

	if(BufferStatusADC1[0] == FULL_ADC_1) {
		BufferStatusADC1[0] = EMPTY_ADC_1;

		uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
		                       UDMA_MODE_PINGPONG,
		                       (void *)(ADC1_BASE + ADC_O_SSFIFO0),
		                       ADC1Buffer1, ADC_SAMPLE_BUF_SIZE);

		uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
	} else if(BufferStatusADC1[1] == FULL_ADC_1) {

		BufferStatusADC1[1] = EMPTY_ADC_1;

		uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
		 		               UDMA_MODE_PINGPONG,
		 		               (void *)(ADC1_BASE + ADC_O_SSFIFO0),
		 		               ADC1Buffer2, ADC_SAMPLE_BUF_SIZE);

		uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
	}

}

//*****************************************************************************

void InitADCs(void)
{
	BufferStatusADC0[0] = FILLING_ADC_0;
	BufferStatusADC0[1] = EMPTY_ADC_0;

	BufferStatusADC1[0] = FILLING_ADC_1;
	BufferStatusADC1[1] = EMPTY_ADC_1;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                    GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

	uDMAEnable();

	uDMAControlBaseSet(ControlTable);

	uDMAChannelAssign(UDMA_CH14_ADC0_0);
	uDMAChannelAssign(UDMA_CH24_ADC1_0);

	uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
	                            UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
	                            UDMA_ATTR_REQMASK);

	uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10,
		                            UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
		                            UDMA_ATTR_REQMASK);

	uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
						  UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 |
						  UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
						  UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT, UDMA_SIZE_16 |
						  UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

	uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
						   UDMA_MODE_PINGPONG,
						   (void *)(ADC0_BASE + ADC_O_SSFIFO0),
						   ADC0Buffer1, ADC_SAMPLE_BUF_SIZE);

	uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
						   UDMA_MODE_PINGPONG,
						   (void *)(ADC0_BASE + ADC_O_SSFIFO0),
						   ADC0Buffer2, ADC_SAMPLE_BUF_SIZE);

	uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
						   UDMA_MODE_PINGPONG,
						   (void *)(ADC1_BASE + ADC_O_SSFIFO0),
						   ADC1Buffer1, ADC_SAMPLE_BUF_SIZE);

	uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
						   UDMA_MODE_PINGPONG,
						   (void *)(ADC1_BASE + ADC_O_SSFIFO0),
						   ADC1Buffer2, ADC_SAMPLE_BUF_SIZE);

	uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);
	uDMAChannelAttributeEnable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_USEBURST);

	uDMAChannelEnable(UDMA_CHANNEL_ADC0);
	uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

	//ADCHardwareOversampleConfigure(ADC0_BASE, 8);
	//ADCHardwareOversampleConfigure(ADC1_BASE, 8);

	// Oversampling average multiple samples from same analog input. Rates suported are
	// 2x, 4x, 8x, 16x, 32x, 64x. If set to 0 hardware oversampling is disabled

	ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);
	ADCClockConfigSet(ADC1_BASE,ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);

	SysCtlDelay(10);

	IntDisable(INT_ADC0SS0);
	IntDisable(INT_ADC1SS0);

	ADCIntDisable(ADC0_BASE, 0);
	ADCIntDisable(ADC1_BASE, 0);

	ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
	ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V);

	ADCSequenceDisable(ADC0_BASE, 0);
	ADCSequenceDisable(ADC1_BASE, 0);

	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
	ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH5); 							// VOLTAGE_1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH4); 							// VOLTAGE_2
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH6); 							// VOLTAGE_3
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH7); 							// VOLTAGE_4
    ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH13); 							// LV_2X_SIGNAL1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH14); 							// LV_2X_SIGNAL2
    ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH15 | ADC_CTL_END | ADC_CTL_IE); // LV_2X_SIGNAL3

    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3); 							// CURRENT_1
    ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH2); 							// CURRENT_2
    ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_CH1); 							// CURRENT_3
    ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH12); 							// CURRENT_4
    ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_CH16); 							// DRIVER_VOLT
    ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADC_CTL_CH17); 							// DRIVER2_AMP
    ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADC_CTL_CH18 | ADC_CTL_END | ADC_CTL_IE); // DRIVER1_AMP

	ADCSequenceEnable(ADC0_BASE, 0);
	ADCSequenceEnable(ADC1_BASE, 0);

	ADCSequenceDMAEnable(ADC0_BASE, 0);
	ADCSequenceDMAEnable(ADC1_BASE, 0);

	ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0);
	ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0);

	IntEnable(INT_ADC0SS0);
	IntEnable(INT_ADC1SS0);

	IntMasterEnable();

	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

	TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/16000) - 1);

	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

	TimerEnable(TIMER0_BASE, TIMER_A);

}

//*****************************************************************************

int main(void)
{
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	SysCtlDelay(20);

	InitADCs();

	while(1)
	{

//*****************************************************************************

		VoltageCh1.Adc_Value = ADC0Buffer1[0];
		VoltageCh1.Adc_Value = VoltageCh1.Adc_Value - 2048.0;
		VoltageCh1.Value = (((float)VoltageCh1.Adc_Value) * (10.0/2048));

//*****************************************************************************

		VoltageCh2.Adc_Value = ADC0Buffer1[1];
		VoltageCh2.Adc_Value = VoltageCh2.Adc_Value - 2048.0;
		VoltageCh2.Value = (((float)VoltageCh2.Adc_Value) * (10.0/2048));

//*****************************************************************************

		VoltageCh3.Adc_Value = ADC0Buffer1[2];
		VoltageCh3.Adc_Value = VoltageCh3.Adc_Value - 2048.0;
		VoltageCh3.Value = (((float)VoltageCh3.Adc_Value) * (10.0/2048));

//*****************************************************************************

		VoltageCh4.Adc_Value = ADC0Buffer1[3];
		VoltageCh4.Adc_Value = VoltageCh4.Adc_Value - 2048.0;
		VoltageCh4.Value = (((float)VoltageCh4.Adc_Value) * (10.0/2048));

//*****************************************************************************

		LvCurrentCh1.Adc_Value = ADC0Buffer1[4];
		LvCurrentCh1.Adc_Value = LvCurrentCh1.Adc_Value - 2048.0;
		LvCurrentCh1.Value = (((float)LvCurrentCh1.Adc_Value) * ((CurrentRange(34.5, 0.025, 120.0, 3.0)) / (2048.0)));

//*****************************************************************************

		LvCurrentCh2.Adc_Value = ADC0Buffer1[5];
		LvCurrentCh2.Adc_Value = LvCurrentCh2.Adc_Value - 2048.0;
		LvCurrentCh2.Value = (((float)LvCurrentCh2.Adc_Value) * ((CurrentRange(34.5, 0.025, 120.0, 3.0)) / (2048.0)));

//*****************************************************************************

		LvCurrentCh3.Adc_Value = ADC0Buffer1[6];
		LvCurrentCh3.Adc_Value = LvCurrentCh3.Adc_Value - 2048.0;
		LvCurrentCh3.Value = (((float)LvCurrentCh3.Adc_Value) * ((CurrentRange(34.5, 0.025, 120.0, 3.0)) / (2048.0)));

//*****************************************************************************

		CurrentCh1.Adc_Value = ADC1Buffer1[0];
		CurrentCh1.Adc_Value = CurrentCh1.Adc_Value - 2048.0;
		CurrentCh1.Value = (((float)CurrentCh1.Adc_Value) * ((CurrentRange(130.0, 0.130, 50.0, 7.5)) / (2048.0)));

//*****************************************************************************

		CurrentCh2.Adc_Value = ADC1Buffer1[1];
		CurrentCh2.Adc_Value = CurrentCh2.Adc_Value - 2048.0;
		CurrentCh2.Value = (((float)CurrentCh2.Adc_Value) * ((CurrentRange(130.0, 0.130, 50.0, 7.5)) / (2048.0)));

//*****************************************************************************

		CurrentCh3.Adc_Value = ADC1Buffer1[2];
		CurrentCh3.Adc_Value = CurrentCh3.Adc_Value - 2048.0;
		CurrentCh3.Value = (((float)CurrentCh3.Adc_Value) * ((CurrentRange(130.0, 0.130, 50.0, 7.5)) / (2048.0)));

//*****************************************************************************

		CurrentCh4.Adc_Value = ADC1Buffer1[3];
		CurrentCh4.Adc_Value = CurrentCh4.Adc_Value - 2048.0;
		CurrentCh4.Value = (((float)CurrentCh4.Adc_Value) * ((CurrentRange(130.0, 0.130, 50.0, 7.5)) / (2048.0)));

//*****************************************************************************

		DriverVolt.Adc_Value = ADC1Buffer1[4];
		DriverVolt.Value = (float)DriverVolt.Adc_Value * 0.00439453125;

//*****************************************************************************

		Driver2Curr.Adc_Value = ADC1Buffer1[5];
		Driver2Curr.Adc_Value = Driver2Curr.Adc_Value - 2048.0;
		Driver2Curr.Value = (float)Driver2Curr.Adc_Value * 0.003662109375;

//*****************************************************************************

		Driver1Curr.Adc_Value = ADC1Buffer1[6];
		Driver1Curr.Adc_Value = Driver1Curr.Adc_Value - 2048.0;
		Driver1Curr.Value = (float)Driver1Curr.Adc_Value * 0.003662109375;

//*****************************************************************************

	}
}

//*****************************************************************************






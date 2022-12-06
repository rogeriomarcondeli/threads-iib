/*
 * main-adc-udma-2.c
 *
 *  Created on: 25 de nov de 2022
 *      Author: rogerio.marcondeli
 */

#include <stdint.h>
#include <stdbool.h>
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
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/emac.h"
#include "driverlib/timer.h"

#pragma DATA_ALIGN(pui8ControlTable, 1024)

uint8_t pui8ControlTable[1024];

#define MEM_BUFFER_SIZE         8

static uint32_t g_ui8RxBufA[MEM_BUFFER_SIZE];
static uint32_t g_ui8RxBufB[MEM_BUFFER_SIZE];

volatile uint32_t g_ui32SysClock;

float Gain = 0.00439453125; // 18V/4096

float Value = 0.0;

static int Adc_Value = 0;

void ADCconfigure();

//*****************************************************************************
void uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    ui32Status = uDMAErrorStatusGet();	// Check for uDMA error bit

    if(ui32Status)						// If there is a uDMA error
    {
        uDMAErrorStatusClear();			//Clear the error status
    }

}
//*****************************************************************************
void ADCseq0Handler(void)
{
	uint32_t ui32Status = ADCIntStatus(ADC0_BASE, 0, false);
    uint32_t ui32Mode;

    ADCIntClear(ADC0_BASE, 0);						//se puede eliminar
    ADCIntClearEx(ADC0_BASE, ADC_INT_DMA_SS0);


    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);

    if(ui32Mode == UDMA_MODE_STOP)

    {
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                                   UDMA_MODE_PINGPONG,
                                   (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                   g_ui8RxBufA, MEM_BUFFER_SIZE);
    }

    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);

    if(ui32Mode == UDMA_MODE_STOP)

    {
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                                    UDMA_MODE_PINGPONG,
                                    (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                    g_ui8RxBufB, MEM_BUFFER_SIZE);
    }

    uDMAChannelEnable(UDMA_CHANNEL_ADC0);
}
//*****************************************************************************
void ADCconfigure()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	SysCtlDelay(10); // Time for the peripheral enable to set

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

	SysCtlDelay(30);

	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);

	SysCtlDelay(80);

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    //ADCHardwareOversampleConfigure(ADC0_BASE, 64);

    // Oversampling average multiple samples from same analog input. Rates suported are
    // 2x, 4x, 8x, 16x, 32x, 64x. If set to 0 hardware oversampling is disabled

    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);  //Clock source (Precision Internal Clock)


    ADCSequenceConfigure(ADC0_BASE, 0 /*SS0*/, ADC_TRIGGER_TIMER, 0 /*priority*/);  // SS0-SS3 priorities must always be different

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH16); // VOLTAGE_2
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH5); // VOLTAGE_1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH6); // VOLTAGE_3
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH7); // VOLTAGE_4
    ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH12); // CURRENT_4
    ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH13); // LV_2X_SIGNAL1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH14); // LV_2X_SIGNAL2
    ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH4 | ADC_CTL_END | ADC_CTL_IE); // DRIVER_VOLT

    ADCSequenceEnable(ADC0_BASE, 0);
    ADCSequenceDMAEnable(ADC0_BASE, 0);

    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                              UDMA_ARB_256);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                            UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
                            UDMA_ARB_256);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                               g_ui8RxBufA, MEM_BUFFER_SIZE);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                                UDMA_MODE_PINGPONG,
                                (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                                g_ui8RxBufB, MEM_BUFFER_SIZE);

    uDMAChannelEnable(UDMA_CHANNEL_ADC0);
    ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS0);
    IntEnable(INT_ADC0SS0);
}

int main(void)
{
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                      SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                        SYSCTL_CFG_VCO_480), 120000000);


     SysCtlDelay(10);        					// Delay for a bit.

     SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

     SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //teste

     TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC); //teste

     TimerLoadSet(TIMER0_BASE, TIMER_A, (g_ui32SysClock/16000) - 1); //teste

     TimerControlTrigger(TIMER0_BASE, TIMER_A, true); //teste

     TimerEnable(TIMER0_BASE, TIMER_A); //teste

     uDMAEnable();

     uDMAControlBaseSet(pui8ControlTable);

     ADCconfigure();

     IntMasterEnable();

     IntEnable(INT_UDMAERR);

     while(1)
     {
    	 Adc_Value = g_ui8RxBufA[0];
    	 Value = (float)Adc_Value * Gain;
     }
}

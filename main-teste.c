/*
 * main-teste.c
 *
 *  Created on: 23 de nov de 2022
 *      Author: rogerio.marcondeli
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_uart.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include <driverlib/interrupt.c>
#include <driverlib/sysctl.c>
#include <driverlib/timer.c>
#include <driverlib/udma.c>
#include <driverlib/gpio.c>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>

void InituDMA(void);
void InitGPIO(void);
void InitTimer(void);
void TimerInt(void);

/*Here you can define what GPIO output you want to use */

#define GPIO_BASE_OUTPUT1 GPIO_PORTE_BASE
#define GPIO_PERIPH_OUTPUT1 SYSCTL_PERIPH_GPIOE

volatile uint32_t g_ui32SysClock; //Saves the system clk

static uint8_t OutputState[2]; //Array to save the GPIO states

/////////////////////////////////////////////////////////
// The control table used by the uDMA controller.      //
// This table must be aligned to a 1024 byte boundary. //
/////////////////////////////////////////////////////////

#if defined(ewarm)

#pragma data_alignment=1024
uint8_t DMAcontroltable[1024];

#elif defined(ccs)

#pragma DATA_ALIGN(DMAcontroltable, 1024)
uint8_t DMAcontroltable[1024];

#else

uint8_t DMAcontroltable[1024] __attribute__ ((aligned(1024)));

#endif

/* After 2 transfers the DMA is done and calls this interrupt   This is to reset the DMA and re-enable it */

void TimerInt(void){

	TimerIntClear(TIMER3_BASE,TIMER_TIMA_DMA); //Set again the same source address and destination

	uDMAChannelTransferSet(UDMA_CH2_TIMER3A | UDMA_PRI_SELECT, UDMA_MODE_BASIC, OutputState, (void *)(GPIO_BASE_OUTPUT1 + 0x3FC),2); //Always needed since after it's done the DMA is disabled when in basic mode

	uDMAChannelEnable(UDMA_CH2_TIMER3A);

}

/* Function to setup the DMA */

void InituDMA(){

	SysCtlPeripheralDisable(SYSCTL_PERIPH_UDMA); //Just disable to be able to reset the peripheral state

	SysCtlPeripheralReset(SYSCTL_PERIPH_UDMA);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

	SysCtlDelay(10);

	uDMAEnable();

	uDMAControlBaseSet(DMAcontroltable);

	/* This is for seting up the GPIO_BASE_OUTPUT1 with CH2 TimerA  */

	uDMAChannelAssign(UDMA_CH2_TIMER3A); //Set the channel trigger to be Timer3A

	uDMAChannelAttributeDisable(UDMA_CH2_TIMER3A, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK); //Disable all the atributes in case any was set

	/* This sets up the item size to 8bits, source increment to 8bits and destination increment to none and arbitration size to 1 */

	uDMAChannelControlSet(UDMA_CH2_TIMER3A | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);

	/* This will setup the transfer mode to basic, source address to the array we want and destination address to the GPIO state we chosed. It also sets the total transfer size to 2. */

	uDMAChannelTransferSet(UDMA_CH2_TIMER3A | UDMA_PRI_SELECT, UDMA_MODE_BASIC, OutputState, (void *)(GPIO_BASE_OUTPUT1 + 0x3FC), 2);

	uDMAChannelEnable(UDMA_CH2_TIMER3A); //Enable the DMA channel

}

/* This is to set all the pins of the GPIO chosen to output */

void InitGPIO(){

	SysCtlPeripheralDisable(GPIO_PERIPH_OUTPUT1);
	SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOJ);

	SysCtlPeripheralReset(GPIO_PERIPH_OUTPUT1);
	SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOJ);

	SysCtlPeripheralEnable(GPIO_PERIPH_OUTPUT1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

	SysCtlDelay(10);

	GPIOPinTypeGPIOOutput(GPIO_BASE_OUTPUT1, GPIO_PIN_5);

	GPIOPinWrite(GPIO_BASE_OUTPUT1, GPIO_PIN_5, GPIO_PIN_5);

	GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE , GPIO_PIN_0);

	GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_PIN_0);

}

/* Function to setup the timer to count down periodic with 1 second period.   It also enables the DMA trigger and the event to timeout (counter reach 0) */

void InitTimer(){

	SysCtlPeripheralDisable(SYSCTL_PERIPH_TIMER3);

	SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

	SysCtlDelay(10);

	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

	TimerLoadSet(TIMER3_BASE, TIMER_A, 120000000-1);

	TimerIntClear(TIMER3_BASE,TIMER_TIMA_DMA);

	TimerIntRegister(TIMER3_BASE,TIMER_A,TimerInt);

	TimerIntEnable(TIMER3_BASE,TIMER_TIMA_DMA);

	TimerDMAEventSet(TIMER3_BASE,TIMER_DMA_TIMEOUT_A);

}

void main(){

	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000); //Set CLK to 120Mhz


	InitTimer();

	InitGPIO();

	InituDMA();

	/* Sets the pins state all to 0 first and all to 1 in second.
	You can control individual pins in each bit of each array member.
	You can make the array bigger if you want but remember to set the DMA
	total transfer size too */

	OutputState[0]=0x00;
	OutputState[1]=0xFF;

	TimerEnable(TIMER3_BASE,TIMER_A); //Enable the timer to start counting

	//Infinite loop and just watch the pins toggle

	while(1){

	}

}


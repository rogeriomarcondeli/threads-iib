/*
 * main.cpp
 *
 *  Created on: 8 de jul de 2022
 *      Author: rogerio.marcondeli
 */

/////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <inc/hw_sysctl.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>

#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>

/////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t ui32SysClock;

/////////////////////////////////////////////////////////////////////////////////////////////

class THR
{
	public:

	void Toggle_Pin(uint32_t base, uint8_t pins, uint32_t *ms)
	{
		uint32_t value = 0;

		value = GPIOPinRead(base, pins);
		value ^= pins;
		GPIOPinWrite(base, pins, value);

		*ms = 0;
	}


	private:

};

/////////////////////////////////////////////////////////////////////////////////////////////

THR thread1;
THR thread2;
THR thread3;

/////////////////////////////////////////////////////////////////////////////////////////////

void IntTimer1msHandler(void)
{
    static uint32_t thr1 = 0,
    				thr2 = 0,
					thr3 = 0;

    thr1++;
    thr2++;
    thr3++;

    // Clear the timer 1 interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if(thr1 == 100)
    {
    	thread1.Toggle_Pin(GPIO_PORTB_BASE, GPIO_PIN_4, &thr1);
    }

    if(thr2 == 200)
    {
    	thread2.Toggle_Pin(GPIO_PORTB_BASE, GPIO_PIN_5, &thr2);
    }

    if(thr3 == 300)
    {
    	thread3.Toggle_Pin(GPIO_PORTE_BASE, GPIO_PIN_5, &thr3);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * main.c
 */

int main(void)
{
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                       SYSCTL_XTAL_25MHZ | SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOJ);

    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOJ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE , GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE , GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE , GPIO_PIN_0);

    GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_PIN_0);

    // Disable timer 1 peripheral
    SysCtlPeripheralDisable(SYSCTL_PERIPH_TIMER1);

    // Reset timer 1 peripheral
    SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER1);

    // Enable timer 1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Wait for the timer 1 peripheral to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));

    // Disable the timer 1 module.
    TimerDisable(TIMER1_BASE, TIMER_A);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, (ui32SysClock / 1000) - 1);

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER1_BASE, TIMER_A, IntTimer1msHandler);
    IntPrioritySet(INT_TIMER1A, 1);

    // Enable the timer 1.
    TimerEnable(TIMER1_BASE, TIMER_A);

    while(1)
    {

    }

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////





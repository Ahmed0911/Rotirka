/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

#include "Drivers/DBGLed.h"
#include "Drivers/Timer.h"
#include "Drivers/PWMDrv.h"
#include "Drivers/SerialDriver.h"
#include "Drivers/ADCDrv.h"
#include "Drivers/EtherDriver.h"
#include "Drivers/EEPROMDrv.h"
#include "Drivers/WpnOutputs.h"
#include "Drivers/swupdate.h"

#include "CommData.h"
#include "CRC32.h"
//#include "Comm433MHz.h"

uint32_t g_ui32SysClock;
extern bool g_swUpdateRequest;

// Drivers
DBGLed dbgLed;
Timer timerLoop;

SerialDriver serialU2;
SerialDriver serialU3;
SerialDriver serialU5;
ADCDrv adcDrv;
EtherDriver etherDrv;
CRC32 crc;
WpnOutputs wpnOutputs;
PWMDrv pwmOut;

// Global Data
int MainLoopCounter;
float PerfLoopTimeMS;
float PerfCpuTimeMS;
float PerfCpuTimeMSMAX;

SCommEthData GlobalData;

bool PingedSendData = false;

// Systick
#define SysTickFrequency 1000
volatile bool SysTickIntHit = false;

int PWMVAL = 1000;
void main(void)
{
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	FPULazyStackingEnable();

	// Ensure that ext. osc is used!
	SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

	// set clock
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Init
	dbgLed.Init();
	timerLoop.Init();
	crc.Init();
	adcDrv.Init();
	wpnOutputs.Init();
	pwmOut.Init();
	etherDrv.Init();
	serialU5.Init(UART5_BASE, 115200);

	memset(&GlobalData, 0, sizeof(GlobalData));

	// Systick
	SysTickPeriodSet(g_ui32SysClock/SysTickFrequency);
	SysTickIntEnable();
	SysTickEnable();

	// Master INT Enable
	IntMasterEnable();

	while(1)
	{
		timerLoop.Start(); // start timer
		GlobalData.LoopCounter++;

		// Check SW Update Request
		if( g_swUpdateRequest)
		{
		    SoftwareUpdateBegin(g_ui32SysClock);
		}

		/////////////////////////////////
		// INPUTS
		/////////////////////////////////
		// ADC + Current Calc
		adcDrv.Update();
		etherDrv.Process(1);


		GlobalData.BatteryVoltage = adcDrv.BATTVoltage();
		GlobalData.RangeADCValue = adcDrv.GetValue(ADCBATTCURRENT);


		////////////////////////////////////
		// PROCESS
		///////////////////////////////////


		/////////////////////////////////
		// OUTPUTS
		/////////////////////////////////
		// Trap State
        wpnOutputs.Set(WPNOUT1, GlobalData.LightActive);

			// DBG LED (blink at 1 Hz)
		if( (GlobalData.LoopCounter % (SysTickFrequency/2)) == 0) dbgLed.Toggle();

		// Get CPU Time
		PerfCpuTimeMS = timerLoop.GetUS()/1000.0f;
		if( PerfCpuTimeMS > PerfCpuTimeMSMAX ) PerfCpuTimeMSMAX = PerfCpuTimeMS;
		// wait next
		while(!SysTickIntHit);
		SysTickIntHit = false;
		// Get total loop time
		PerfLoopTimeMS = timerLoop.GetUS()/1000.0f;
	}
}

void ProcessCommand(int cmd, unsigned char* data, int dataSize)
{
    switch( cmd )
    {
        case 30: // Set State
        {
            GlobalData.LightActive = data[0];

            break;
        }
    }
}

///////////////
// INTERRUPTS
///////////////
extern "C" void UART2IntHandler(void)
{
	serialU2.IntHandler();
}

extern "C" void UART3IntHandler(void)
{
	serialU3.IntHandler();
}

extern "C" void UART5IntHandler(void)
{
	serialU5.IntHandler();
}

extern "C" void IntGPIOA(void)
{
	//lsm90Drv.MotionINTG();
	//lsm90Drv.MotionINTX();
}

extern "C" void IntGPIOH(void)
{
	//lsm90Drv.MotionINTM();
}

extern "C" void IntGPIOK(void)
{
	//mpu9250Drv.MotionINT();
}

extern "C" void IntGPION(void)
{
	//hopeRF.IntHandler();
}

extern "C" void SysTickIntHandler(void)
{
	SysTickIntHit = true;
}

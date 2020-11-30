/*
 * CommData.h
 *
 *  Created on: May, 5, 2019
 *      Author: John
 */

#ifndef CATTRAP_COMMDATA_H_
#define CATTRAP_COMMDATA_H_

enum STrapState { Off = 0, Armed, Activated };

struct SCommEthData
{
	unsigned int LoopCounter;

	// General Status
	float BatteryVoltage; // [V]
	unsigned int RangeADCValue; // [0...4095]

	// State Machine
	unsigned short LightActive; // [0...1]
};

#endif /* CATTRAP_COMMDATA_H_ */

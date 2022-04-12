/**
  ******************************************************************************
  * @file           : SM72245.cpp
  * @brief          : Driver for SM7245 MPPT Controller
  * @author			: Lawrence Stanton
  * @revised		: 11 Apr 2022
  ******************************************************************************
  * @attention
  *
  * © LD Stanton 2022
  * 
  * This file and its content are the copyright property of the author. All 
  * rights are reserved. No warranty is given. No liability is assumed.
  * Confidential unless licensed otherwise. If licensed, refer to the 
  * accompanying file "LICENCE" for license details.
  * 
  ******************************************************************************
  */

/* Begin Private Includes */
#include "SM72445.hpp"

/* End Private Includes */

/* Begin Private Defines */


/* End Private Defines */

/* Begin Private Macros */


/* End Private Macros */

/* Begin Private Typedefs */


/* End Private Typedefs */

/* Begin Private Function Prototypes */


/* End Private Function Prototypes */

/* Begin Private Variables */


/* End Private Variables */

/* Begin Source Code */


bool SM72445::isWritableRegister(RegisterAddr reg){
	if( (reg != SM72445::RegisterAddr::REG0) && (reg != SM72445::RegisterAddr::REG1) ) return true;
	else return false;
}


SM72445::Thresholds::Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow){
	const uint16_t mask = 0x03FFu;
	this->iInHigh  = (mask & iInHigh)	? 0x0000u : iInHigh;
	this->iInLow   = (mask & iInLow)	? 0x0000u : iInLow;
	this->iOutHigh = (mask & iOutHigh)	? 0x0000u : iOutHigh;
	this->iOutLow  = (mask & iOutLow)	? 0x0000u : iOutLow;
}

SM72445::ADCs::ADCs(uint16_t ADC0, uint16_t ADC2, uint16_t ADC4, uint16_t ADC6){
	const uint16_t mask = 0x03FFu;
	this->ADC0 = (mask & ADC0) ? 0x0000u : ADC0;
	this->ADC2 = (mask & ADC2) ? 0x0000u : ADC2;
	this->ADC4 = (mask & ADC4) ? 0x0000u : ADC4;
	this->ADC6 = (mask & ADC6) ? 0x0000u : ADC6;
}

SM72445::VoltsCurrents::VoltsCurrents(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut, const uint16_t mask){
	this->vIn  = (mask & vIn)  ? 0x0000u : vIn;
	this->vOut = (mask & vOut) ? 0x0000u : vOut;
	this->iIn  = (mask & iIn)  ? 0x0000u : iIn;
	this->iOut = (mask & iOut) ? 0x0000u : iOut;
}

SM72445::Offsets::Offsets(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut) :
	VoltsCurrents(vIn,vOut, iIn, iOut, 0x00FFu){}

SM72445::Sensors::Sensors(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut) :
	VoltsCurrents(vIn,vOut, iIn, iOut, 0x03FFu){}




/* End Source Code */


/*** END OF FILE ***/

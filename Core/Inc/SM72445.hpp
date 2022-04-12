/**
  ******************************************************************************
  * @file           : SM72245.hpp
  * @brief          : Header file for SM72445.cpp
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

#ifndef INC_SM72245_HPP_
#define INC_SM72245_HPP_

/* Begin Public Includes */
#pragma once
#include <stdio.h>
/* End Public Includes */

/* Begin Public Defines */
#define SM72445_REG_SIZE 7

/* End Public Defines */

/* Begin Public Typedefs */


/* End Public Typedefs */

/* Begin Global Public Variables */


/* End Global Public Variables */

/* Begin Public Function Prototypes */


/* End Public Function Prototypes */

/* Begin Public Class Definitions */
class SM72445{
	private:
	struct VoltsCurrents;
	struct Sensors;
	struct Offsets;
	struct Thresholds;
	struct ADCs;

	enum class ReturnType : int;

	public:
	enum class I2CAddr : uint8_t;
	enum class RegisterAddr : uint8_t;
	
	typedef struct Sensors SensorsTypedef;
	typedef struct Offsets OffsetsTypedef;
	typedef struct Thresholds ThresholdsTypedef;
	typedef struct ADCs ADCsTypedef;


	SM72445();
	~SM72445();

	SensorsTypedef getSensors();
	OffsetsTypedef getOffsets();

	void setOffsets(OffsetsTypedef offsets);
	void setThresholds(ThresholdsTypedef thresholds);


	private:
	I2CAddr i2cAddr;

	static bool isWritableRegister(RegisterAddr reg);

	static SM72445::ReturnType I2C_MemRead(uint8_t i2cAddr, uint8_t regAddr, uint8_t & pData, uint16_t size);
	static SM72445::ReturnType I2C_MemWrite(uint8_t i2cAddr, uint8_t regAddr, uint8_t & pData, uint16_t size);

};

enum class SM72445::I2CAddr : uint8_t{
	ADDR1 = 0x1u,
	ADDR2 = 0x2u,
	ADDR3 = 0x3u,
	ADDR4 = 0x4u,
	ADDR5 = 0x5u,
	ADDR6 = 0x6u,
	ADDR7 = 0x7u
};

enum class SM72445::RegisterAddr : uint8_t{
	REG0 = 0xE0u,
	REG1 = 0xE1u,
	REG2 = 0xE2u,
	REG3 = 0xE3u,
	REG4 = 0xE4u,
	REG5 = 0xE5u
};

enum class SM72445::ReturnType : int{
	OK = 0,
	FAIL_I2C = -1
};

struct SM72445::VoltsCurrents{ 
	volatile uint16_t vIn;
	volatile uint16_t vOut;
	volatile uint16_t iIn;
	volatile uint16_t iOut;
	
	VoltsCurrents(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut, const uint16_t mask);
};

struct SM72445::Offsets : virtual VoltsCurrents{ Offsets(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut); };
struct SM72445::Sensors : virtual VoltsCurrents{ Sensors(uint16_t vIn, uint16_t vOut, uint16_t iIn, uint16_t iOut); };
struct SM72445::ADCs{
	uint16_t ADC0;
	uint16_t ADC2;
	uint16_t ADC4;
	uint16_t ADC6;

	ADCs(uint16_t ADC0, uint16_t ADC2, uint16_t ADC4, uint16_t ADC6);
};

struct SM72445::Thresholds{
	uint16_t iInHigh;
	uint16_t iInLow;
	uint16_t iOutHigh;
	uint16_t iOutLow;

	Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow);
};


/* End Public Class Definitions */

#endif /* INC_SM72245_HPP_ */

/*** END OF FILE ***/

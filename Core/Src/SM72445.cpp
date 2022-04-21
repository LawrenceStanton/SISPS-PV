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
#define SM72445_VDDA 5.0f
#define SM72445_ADC__FS_RANGE 1023u

#define SM72445_CONFIG_ADC_OVERRIDE_BIT 	(0b1u << 46)
#define SM72445_CONFIG_PM_OVERRIDE_BIT		(0b1u << 4)
#define SM72445_CONFIG_PM_SET_BIT			(0b1u << 3)
#define SM72445_CONFIG_RESET_BIT			(0b1u << 2)


#define SM72445_CONFIG_FREQ_PM_OFFSET	40u
#define SM72445_CONFIG_IOUTMAX_OFFSET	30u
#define SM72445_CONFIG_VOUTMAX_OFFSET	20u
//#define SM72445_CONFIG_TDOFF_OFFSET		17u
//#define SM72445_CONFIG_TDON_OFFSET		14u


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

SM72445::SM72445(I2CAddr i2cAddr, float vInGain, float vOutGain, float iInGain, float iOutGain ){
	this->i2cAddr = i2cAddr;
	this->vInGain = vInGain;
	this->vOutGain = vOutGain;
	this->iInGain = iInGain;
	this->iOutGain = iOutGain;
}


SM72445::ThresholdsTypedef SM72445::getThresholds(){
	uint64_t reg5;
	SM72445::I2C_MemRead((uint8_t)this->i2cAddr, (uint8_t)SM72445::RegisterAddr::REG5, reg5);

	return SM72445::Thresholds(reg5);
}


bool SM72445::isWritableRegister(RegisterAddr reg){
	if( (reg != SM72445::RegisterAddr::REG0) && (reg != SM72445::RegisterAddr::REG1) ) return true;
	else return false;
}

template <typename U>
SM72445::QuadDataStruct<U>::QuadDataStruct(){
	this->bitsize = 0;
	for(int i = 0; i < 4; i++) this->data[i] = 0;
}

template <typename U>
SM72445::QuadDataStruct<U>::QuadDataStruct(const uint8_t bitsize, U u1, U u2, U u3, U u4){
	this->bitsize = bitsize;

	this->data[0] = u1;
	this->data[1] = u2;
	this->data[2] = u3;
	this->data[3] = u4;
}

template <typename U>
SM72445::QuadDataStruct<U>::QuadDataStruct(const uint8_t bitsize, const uint64_t bigData){
	const U mask = (0b1u << bitsize) - 1;

	for(int i = 0; i < 4; i++) this->data[i] = ( bigData >> (bitsize*i) ) & mask;
}

SM72445::Thresholds::Thresholds() :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct() {}

SM72445::Thresholds::Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, iInHigh, iInLow, iOutHigh, iOutLow) {}

SM72445::Thresholds::Thresholds(uint64_t bigData) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, bigData) {}


/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific I2C and Delay Implementations
 * For convenience, the below methods are methods are suitable implementations of the I2C communication and delay methods
 * for STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL) for systems with a single
 * device or multiple SM72445 devices on a common I2C bus.
 *************************************************************************************************************************/

#define HDC1080_USE_STM32_HAL_METHODS
#ifdef HDC1080_USE_STM32_HAL_METHODS

#include "stm32g0xx_hal.h"	// STM32 HAL driver to be included. Adjust depending on the specific STM32 platform.

extern I2C_HandleTypeDef * sm72445_hi2c;	// Pointer to the HAL I2C handle for the I2C interface that the SM72445 is on.
											// Define this variable in main.cpp.

SM72445::Status SM72445::I2C_MemRead (uint8_t i2cAddr, uint8_t regAddr, uint64_t & rData){
	uint8_t bytes[SM72445_REG_SIZE];
	HAL_StatusTypeDef halResult =  HAL_I2C_Mem_Read(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE, 500);

	rData = 0u;
	for(int i = SM72445_REG_SIZE; i > 0; i--) rData = (rData<<8) + bytes[i];

	return (halResult == HAL_OK) ? SM72445::Status::OK : SM72445::Status::FAIL_I2C;
}

SM72445::Status SM72445::I2C_MemWrite(uint8_t i2cAddr, const uint8_t regAddr, uint64_t data){
	uint8_t bytes[SM72445_REG_SIZE];
	const uint8_t mask = 0xFFu;

	for(int i = 0; i < SM72445_REG_SIZE; i++) bytes[i] = (data >> 8*i) & mask;

	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE, 500);
	return (halResult == HAL_OK) ? SM72445::Status::OK : SM72445::Status::FAIL_I2C;
}

#endif /* HDC1080_USE_STM32_HAL_METHODS */

/* End Source Code */


/*** END OF FILE ***/

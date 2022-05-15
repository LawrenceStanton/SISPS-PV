/**
  ******************************************************************************
  * @file           : TMP116.cpp
  * @brief          : Source file for TMP116 Driver.
  * @author			: Lawrence Stanton
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

/* --------------------------------------------------------------------------- */
/* Begin Private Includes */
#include "TMP116.hpp"
#include <string>

/* End Private Includes */

/* --------------------------------------------------------------------------- */
/* Begin Private Defines */
#define TMP116_HIGH_ALERT_MASK  static_cast<uint16_t>(0b1u   << 15)
#define TMP116_LOW_ALERT_MASK   static_cast<uint16_t>(0b1u   << 14)
#define TMP116_DATA_READY_MASK  static_cast<uint16_t>(0b1u   << 13)
#define TMP116_EEPROM_BUSY_MASK static_cast<uint16_t>(0b1u   << 15)
#define TMP116_MOD_MASK			static_cast<uint16_t>(0b11u  << 10)
#define TMP116_CONV_MASK		static_cast<uint16_t>(0b111u << 7)
#define TMP116_AVG_MASK			static_cast<uint16_t>(0b11u  << 5)
#define TMP116_THERM_ALERT_MASK	static_cast<uint16_t>(0b1u   << 4)
#define TMP116_POL_MASK			static_cast<uint16_t>(0b1u   << 3)
#define TMP116_DR_ALERT_MASK	static_cast<uint16_t>(0b1u   << 2)

#define TMP116_RESOLUTION (7.8125f * 1E-3)

/* End Private Defines */

/* --------------------------------------------------------------------------- */
/* Begin Private Macros */


/* End Private Macros */

/* --------------------------------------------------------------------------- */
/* Begin Private Typedefs */


/* End Private Typedefs */

/* --------------------------------------------------------------------------- */
/* Begin Private Function Prototypes */
/**
 * @brief Lambda functions to be used with I2C_Mutate.
 */
static const auto lambda_set   = [](uint16_t reg, uint16_t bits, uint16_t) 		-> uint16_t { return reg |  bits; };
static const auto lambda_reset = [](uint16_t reg, uint16_t bits, uint16_t) 		-> uint16_t { return reg & ~bits; };
static const auto lambda_write = [](uint16_t reg, uint16_t bits, uint16_t mask) -> uint16_t { return (reg & ~mask) | (mask & bits); };

/* End Private Function Prototypes */

/* --------------------------------------------------------------------------- */
/* Begin Private Variables */


/* End Private Variables */

/* --------------------------------------------------------------------------- */
/* Begin Source Code */

TMP116::TMP116(I2CAddr i2cAddr, std::string proximity){
	this->i2cAddr = i2cAddr;
	this->proximity = proximity;
}

int16_t TMP116::getTemperatureReg(){
	uint16_t reg;
	auto status = I2C_MemRead(this->i2cAddr, RegisterAddr::TEMP, reg);
	return (status == Status::OK) ? static_cast<int16_t>(reg) : -0xFF;
}

float TMP116::getTemperature(){
	auto tReg = getTemperatureReg();
	float T = TMP116_RESOLUTION * tReg;
	return T;
}

uint16_t TMP116::getDevID(){
	uint16_t devID;
	auto i2cResult = I2C_MemRead(this->i2cAddr, RegisterAddr::DEVICE_ID, devID);
	return (i2cResult == Status::OK) ? devID : 0x00u;
}

inline bool TMP116::getHighAlert(){ return getConfigTrue(TMP116_HIGH_ALERT_MASK); }

inline bool TMP116::getLowAlert(){	 return getConfigTrue(TMP116_LOW_ALERT_MASK); }

inline bool TMP116::dataReady(){ 	 return getConfigTrue(TMP116_DATA_READY_MASK); }

inline bool TMP116::eepromBusy(){ 	 return getConfigTrue(TMP116_EEPROM_BUSY_MASK); }

/**
 * @brief General method for checking if the configuration register matches a given mask for certain settings.
 * 
 * @param mask	A mask for the settings to be checked.
 * @return true Match under mask.
 * @return false No match under mask or I2C error.
 */
bool TMP116::getConfigTrue(uint16_t mask){
	uint16_t config;
 	auto status = I2C_MemRead(this->i2cAddr, RegisterAddr::CFGR, config);

	if(status == Status::OK)
		if(config & mask) return true;
	return false;
}

void TMP116::setConversionMode(ConversionMode mode){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(mode), lambda_write, TMP116_MOD_MASK);
}

inline void TMP116::setConversionCycle(ConversionTime conv){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(conv), lambda_write, TMP116_CONV_MASK);
}

inline void TMP116::setAverages(Averages avg){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(avg), lambda_write, TMP116_AVG_MASK);
}

void TMP116::setAlertMode(ThermAlertMode mode){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(mode), lambda_write, TMP116_THERM_ALERT_MASK);
}

inline void TMP116::setAlertPolarity(Polarity pol){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(pol), lambda_write, TMP116_POL_MASK);
}

inline void TMP116::setAlertPinMode(AlertPinMode mode){
	I2C_Mutate(this->i2cAddr, RegisterAddr::CFGR, static_cast<uint16_t>(mode), lambda_write, TMP116_DR_ALERT_MASK);
}

inline void TMP116::setHighLimit(int16_t limit){
	I2C_MemWrite(this->i2cAddr, RegisterAddr::HIGH_LIM, (uint16_t)limit);
}

void TMP116::setHighLimit(float limit){
	if((limit >= -256.0) && (limit <= 192.0))
	setHighLimit((int16_t)(limit * TMP116_RESOLUTION));
}

inline void TMP116::setLowLimit(int16_t limit){
	I2C_MemWrite(this->i2cAddr, RegisterAddr::LOW_LIM, (uint16_t)limit);
}

void TMP116::setLowLimit(float limit){
	if((limit >= -256.0) && (limit <= 192.0))
	setLowLimit((int16_t)(limit * TMP116_RESOLUTION));
}

/**
 * @brief General purpose function used to modify certain parts of an I2C register while maintaining all other bits.
 * 
 * @tparam T The type of function used for the critical action.
 * @param i2cAddr The I2C address of the device to act upon.
 * @param regAddr The address of the internal register to act upon.
 * @param bits The bitpattern of the desired modification.
 * @param action The critical action to perform upon the register. e.g. Set bits, reset bits, write bits, etc.
 * @param mask Optional and if necessary to specify all bits that are to be modified, if required by action.
 * @return uint64_t The final value written to the I2C register.
 * 
 * @note A return of 0x0ull may indicate a I2C communication failure, however this value may have not necessarily been written.
 */
template<typename T>
uint16_t TMP116::I2C_Mutate(I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t bits, T&& action, uint16_t mask){
	uint16_t reg;

	if(I2C_MemRead(i2cAddr, regAddr, reg) == Status::OK){
		reg = action(reg, bits, mask);
		if(I2C_MemWrite(i2cAddr, regAddr, reg) == Status::OK)	return reg;
	}
	return 0x00u;
}

#define TMP116_USE_STM32_HAL_METHODS
#ifdef TMP116_USE_STM32_HAL_METHODS

/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific I2C and Delay Implementations
 * For convenience, the below methods are methods are suitable implementations of the I2C communication and delay methods
 * for STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL) for a TMP116s on a single
 * I2C bus.
 *************************************************************************************************************************/

#include "stm32g0xx_hal.h"	// STM32 HAL driver to be included. Adjust depending on the specific STM32 platform.

extern I2C_HandleTypeDef * tmp116_hi2c;	// Pointer to the HAL I2C handle for the I2C interface that the TMP116s is on.
										// Define this variable in main.cpp.

TMP116::Status TMP116::I2C_MemRead (I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t & rData){
	static_assert(TMP116_REG_SIZE == 2, "Error in TMP116_REG_SIZE.");

	uint8_t bytes[TMP116_REG_SIZE];
	HAL_StatusTypeDef halResult =  HAL_I2C_Mem_Read(tmp116_hi2c, (static_cast<uint8_t>(i2cAddr)<<1), static_cast<uint8_t>(regAddr), 1, bytes, TMP116_REG_SIZE, 500);

	auto smResult = (halResult == HAL_OK) ? Status::OK : Status::FAIL_I2C;

	if(smResult == Status::OK) rData = (bytes[0] << 8) | bytes[1];

	return smResult;
}

TMP116::Status TMP116::I2C_MemWrite(I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t data){
	static_assert(TMP116_REG_SIZE == 2, "Error in TMP116_REG_SIZE.");

	uint8_t bytes[TMP116_REG_SIZE];
	const uint8_t mask = 0xFFu;

	bytes[0] = (data >> 8) & mask;
	bytes[1] = data & mask;

	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(tmp116_hi2c, (static_cast<uint8_t>(i2cAddr)<<1), static_cast<uint8_t>(regAddr), 1, bytes, TMP116_REG_SIZE + 1, 500);
	return (halResult == HAL_OK) ? TMP116::Status::OK : TMP116::Status::FAIL_I2C;
}

#endif /* TMP116_USE_STM32_HAL_METHODS */

/* End Source Code */


/*** END OF FILE ***/

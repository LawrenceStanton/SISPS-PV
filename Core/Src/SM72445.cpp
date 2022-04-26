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
#define SM72445_ADC_FS_RANGE 1023u

#define SM72445_CONFIG_ADC_OVERRIDE_BIT 		( (uint64_t) (0b1ull << 46u) )
#define SM72445_CONFIG_PM_OVERRIDE_BIT			( (uint64_t) (0b1ull <<  4u) )
#define SM72445_CONFIG_PM_SET_BIT				( (uint64_t) (0b1ull <<  3u) )
#define SM72445_CONFIG_RESET_BIT				( (uint64_t) (0b1ull <<  2u) )

#define SM72445_CONFIG_A2_OVERRIDE_MASK			( (uint64_t) (0x007ull << 40u) )
#define SM72445_CONFIG_IOUTMAX_OVERRIDE_MASK	( (uint64_t) (0x3FFull << 30u) )
#define SM72445_CONFIG_VOUTMAX_OVERRIDE_MASK	( (uint64_t) (0x3FFull << 20u) )
//#define SM72445_CONFIG_TDOFF_MASK				( (uint64_t) (0x007ull << 17u) )	//UNUSED
//#define SM72445_CONFIG_TDON_MASK				( (uint64_t) (0x007ull << 14u) )	//UNUSED
//#define SM72445_CONFIG_DC_OPEN_MASK			( (uint64_t) (0x1FFull <<  5u) )	//UNUSED

#define SM72445_CONFIG_A2_OVERRIDE_OFFSET		40u
#define SM72445_CONFIG_IOUTMAX_OVERRIDE_OFFSET	30u
#define SM72445_CONFIG_VOUTMAX_OVERRIDE_OFFSET	20u
//#define SM72445_CONFIG_TDOFF_OFFSET			17u	// UNUSED
//#define SM72445_CONFIG_TDON_OFFSET			14u	// UNUSED
//#define SM72445_CONFIG_DC_OPEN_OFFSET			 5u // UNUSED

/* End Private Defines */

/* Begin Private Macros */


/* End Private Macros */

/* Begin Private Typedefs */


/* End Private Typedefs */

/* Begin Private Function Prototypes */


/* End Private Function Prototypes */

/* Begin Private Variables */

/**
 * @brief Lambda functions to be used with I2C_SetBits and I2C_ResetBits.
 */
static const auto lambda_set   = [](uint64_t reg, uint64_t bits, uint64_t) -> uint64_t { return reg |  bits; };
static const auto lambda_reset = [](uint64_t reg, uint64_t bits, uint64_t) -> uint64_t { return reg & ~bits; };
static const auto lambda_write = [](uint64_t reg, uint64_t bits, uint64_t mask) -> uint64_t { return (reg & ~mask) | (mask & bits); };

/* End Private Variables */

/* Begin Source Code */

SM72445::SM72445(I2CAddr i2cAddr, float vInGain, float vOutGain, float iInGain, float iOutGain ){
	this->i2cAddr = i2cAddr;
	this->vInGain = vInGain;
	this->vOutGain = vOutGain;
	this->iInGain = iInGain;
	this->iOutGain = iOutGain;
}

SM72445::ADCs SM72445::getADC_Values(){
	return get<ADCs>(RegisterAddr::REG0);
}

SM72445::Sensors SM72445::getSensors(){
	return get<Sensors>(RegisterAddr::REG1);
}

SM72445::Offsets SM72445::getOffsets(){
	return get<Offsets>(RegisterAddr::REG4);
}

SM72445::Thresholds SM72445::getThresholds(){
	return get<Thresholds>(RegisterAddr::REG5);
}

template<class T>
T SM72445::get(RegisterAddr regAddr){
	uint64_t reg = 0x0ull;
	I2C_MemRead((uint8_t)this->i2cAddr, (uint8_t)regAddr, reg);

	return T(reg);
}

/**
 * @brief Enables Switching Frequency and Panel Mode Configuration ADC Overriding.
 * 
 * @param fpm	Enumeral for Frequency and Panel Mode setting to override with.
 * 
 * @note This method follows the procedure described on Page 13 of the SM72445 Datasheet.
 */
void SM72445::setADCOverrideFreqPM(FreqPanelMode fpm){
	uint64_t reg;

	if(!this->adcOverride){
		this->adcOverride = true;	// SM72445 ADC registers are now to be overridden.

		reg = I2C_Mutate(
			this->i2cAddr,
			RegisterAddr::REG3,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | ( ((uint64_t)fpm) << SM72445_CONFIG_A2_OVERRIDE_OFFSET),
			lambda_write,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | SM72445_CONFIG_A2_OVERRIDE_MASK);
		
		reg = reg & ~SM72445_CONFIG_RESET_BIT;

		I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg);
	}
	else{
		reg = I2C_Mutate(
			this->i2cAddr,
			RegisterAddr::REG3,
			SM72445_CONFIG_RESET_BIT | ( ((uint64_t)fpm) << SM72445_CONFIG_A2_OVERRIDE_OFFSET),
			lambda_write,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | SM72445_CONFIG_A2_OVERRIDE_MASK);

		reg = reg & ~(SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT);

		if(I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg) == Status::OK){
			reg = reg | SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT;
			
			if(I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg) == Status::OK){
				reg = reg & ~SM72445_CONFIG_RESET_BIT;
				I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg);
			}
		}
	}
}

void SM72445::setADCOverrideMaxOutI(float iOutMax){
	setADCOverrideMaxOutVI(iOutMax, this->iOutGain, SM72445_CONFIG_IOUTMAX_OVERRIDE_OFFSET, SM72445_CONFIG_IOUTMAX_OVERRIDE_MASK);
}

void SM72445::setADCOverrideMaxOutV(float vOutMax){
	setADCOverrideMaxOutVI(vOutMax, this->vOutGain, SM72445_CONFIG_VOUTMAX_OVERRIDE_OFFSET, SM72445_CONFIG_VOUTMAX_OVERRIDE_MASK);
}

void SM72445::setADCOverrideMaxOutVI(float viOutMax, float gain, uint8_t offset, uint64_t mask){
	const float viOutMaxADC = viOutMax * gain;

	if((viOutMax >= 0.0) && (viOutMaxADC < SM72445_VDDA)){
		this->adcOverride = true;	// SM72445 ADC registers are now to be overridden.
	
		uint16_t viOutMaxD = (uint16_t) (SM72445_ADC_FS_RANGE * viOutMaxADC / SM72445_VDDA);
	
		I2C_Mutate(
			this->i2cAddr, 
			RegisterAddr::REG3,
			(((uint64_t)viOutMaxD) << offset) | SM72445_CONFIG_ADC_OVERRIDE_BIT, 
			lambda_write,
			mask | SM72445_CONFIG_ADC_OVERRIDE_BIT);
	}
}

void SM72445::enablePMOverride(){
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_OVERRIDE_BIT, lambda_set);
}

void SM72445::disablePMOverride(){
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_OVERRIDE_BIT, lambda_reset);
}

void SM72445::setPM(bool en){
	auto action = en ? lambda_set : lambda_reset;
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_SET_BIT, action);
}

void SM72445::softReset(){
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_RESET_BIT, lambda_set);
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_RESET_BIT, lambda_reset);
}

void SM72445::setThresholds(Thresholds thresholds){
	set(RegisterAddr::REG5, thresholds);
}

void SM72445::setOffsets(Offsets offsets){
	set(RegisterAddr::REG4, offsets);
}

template<class T>
void SM72445::set(RegisterAddr regAddr, T reg){
	I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)regAddr, reg.toReg() );
}

bool SM72445::isWritableRegister(RegisterAddr reg){
	if( (reg != SM72445::RegisterAddr::REG0) && (reg != SM72445::RegisterAddr::REG1) ) return true;
	else return false;
}

template<typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct() :
	SM72445::QuadDataStruct<T>::QuadDataStruct(0, 0, 0, 0, 0) {}

template <typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct(const uint8_t bitsize, T u1, T u2, T u3, T u4){
	this->bitsize = bitsize;

	this->data[0] = u1;
	this->data[1] = u2;
	this->data[2] = u3;
	this->data[3] = u4;
}

template <typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct(const uint8_t bitsize, const uint64_t reg){
	this->bitsize = bitsize;
	
	const T mask = (0b1u << bitsize) - 1;

	for(int i = 0; i < 4; i++) this->data[i] = ( reg >> (bitsize*i) ) & mask;
}

template<typename T>
uint64_t SM72445::QuadDataStruct<T>::toReg(){
	uint64_t reg = 0x0ul;
	for(int i = 3; i >= 0; i--) reg = (reg << this->bitsize) | this->data[i];
	return reg;
}

template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents() : 
	SM72445::VoltsCurrents<T>::VoltsCurrents(0, 0, 0, 0, 0) {}

template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents(const uint8_t bitsize, T vOut, T iOut, T vIn, T iIn) : 
	SM72445::QuadDataStruct<T>::QuadDataStruct(bitsize, vOut, iOut, vIn, iIn) {}

template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents(const uint8_t bitsize, uint64_t reg) :
	SM72445::QuadDataStruct<T>::QuadDataStruct(bitsize, reg) {}

template <typename T>
SM72445::VoltsCurrents<T> & SM72445::VoltsCurrents<T>::operator = (const VoltsCurrents<T> & vc){
	QuadDataStruct<T>::operator=(vc);
	return * this;
}

SM72445::ADCs::ADCs() : 
	SM72445::ADCs::ADCs(0x0ul) {}

SM72445::ADCs::ADCs(uint64_t reg) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, reg) {}

SM72445::ADCs & SM72445::ADCs::operator=(const ADCs & adc){
	QuadDataStruct<uint16_t>::operator=(adc);
	return * this;
}

SM72445::Sensors::Sensors() : 
	SM72445::VoltsCurrents<uint16_t>::VoltsCurrents(10u, 0, 0, 0, 0) {}

SM72445::Sensors::Sensors(uint64_t reg) : 
	SM72445::VoltsCurrents<uint16_t>::VoltsCurrents(10u, reg) {}

SM72445::Sensors & SM72445::Sensors::operator = (const Sensors & sensor){
	VoltsCurrents<uint16_t>::operator=(sensor);
	return * this;
}

SM72445::Offsets::Offsets(uint8_t vOut, uint8_t iOut, uint8_t vIn, uint8_t iIn) :
	SM72445::VoltsCurrents<uint8_t>::VoltsCurrents(8u, vOut, iOut, vIn, iIn) {}

SM72445::Offsets::Offsets() : 
	SM72445::VoltsCurrents<uint8_t>::VoltsCurrents(8u, 0, 0, 0, 0) {}

SM72445::Offsets::Offsets(uint64_t reg) : 
	SM72445::VoltsCurrents<uint8_t>::VoltsCurrents(8u, reg) {}

SM72445::Offsets & SM72445::Offsets::operator=(const Offsets & offsets){
	VoltsCurrents<uint8_t>::operator=(offsets);
	return * this;
}

SM72445::Thresholds::Thresholds() :
	SM72445::Thresholds::Thresholds(0, 0, 0, 0) {}

SM72445::Thresholds::Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, iInHigh, iInLow, iOutHigh, iOutLow) {}

SM72445::Thresholds::Thresholds(uint64_t reg) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, reg) {}

SM72445::Thresholds & SM72445::Thresholds::operator = (const Thresholds & th){ 
	QuadDataStruct<uint16_t>::operator=(th);
	return * this;
}

template<typename T>
uint64_t SM72445::I2C_Mutate(I2CAddr i2cAddr, RegisterAddr regAddr, uint64_t bits, T&& action, uint64_t mask){
	uint64_t reg;

	if(I2C_MemRead((uint8_t)i2cAddr, (uint8_t)regAddr, reg) == Status::OK){
		reg = action(reg, bits, mask);
		if(I2C_MemWrite((uint8_t)i2cAddr, (uint8_t)regAddr, reg) == Status::OK)	return reg;
	}
	return 0x0ull;
}

/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific I2C Implementations
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
	uint8_t bytes[SM72445_REG_SIZE+1];	// Transmission length byte at byte[0] (unused). See Fig.15 of datasheet.
	HAL_StatusTypeDef halResult =  HAL_I2C_Mem_Read(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE + 1, 500);

	rData = 0u;
	for(int i = SM72445_REG_SIZE + 1; i > 0; i--) rData = (rData<<8) + bytes[i];

	return (halResult == HAL_OK) ? SM72445::Status::OK : SM72445::Status::FAIL_I2C;
}

SM72445::Status SM72445::I2C_MemWrite(uint8_t i2cAddr, const uint8_t regAddr, uint64_t data){
	uint8_t bytes[SM72445_REG_SIZE + 1];
	const uint8_t mask = 0xFFu;

	bytes[0] = SM72445_REG_SIZE;														// Transmission length byte
	for(int i = 1; i < SM72445_REG_SIZE + 1; i++) bytes[i] = (data >> 8*(i-1)) & mask;	// Packing Register Data

	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE + 1, 500);
	return (halResult == HAL_OK) ? SM72445::Status::OK : SM72445::Status::FAIL_I2C;
}

#endif /* HDC1080_USE_STM32_HAL_METHODS */

/* End Source Code */


/*** END OF FILE ***/

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
	protected:
	template <typename T> struct QuadDataStruct;
	template <typename T> struct VoltsCurrents;

	struct Config;
	
	enum class Status : int;

	public:
	enum class I2CAddr : uint8_t;
	enum class RegisterAddr : uint8_t;
	enum class FreqPanelMode : uint8_t;
	
	struct ADCs;
	struct Sensors;
	struct Offsets;
	struct Thresholds;

	SM72445(I2CAddr i2cAddr, float vInGain, float vOutGain, float iInGain, float iOutGain );
	//inline ~SM72445(){ softReset(); }

	ADCs getADC_Values();
	Sensors getSensors();
	Offsets getOffsets();
	Thresholds getThresholds();

	/* The following methods require enableADCOverride to be in effect. */
	void setADCOverrideFreqPM(FreqPanelMode fpm);
	void setADCOverrideMaxOutI(float iOutMax);
	void setADCOverrideMaxOutV(float vOutMax);
	//void setADCOverrideDeadTimeOff();		// Unimplemented
	//void setADCOverrideDeadTimeOn();		// Unimplemented
	
	void enablePMOverride();
	void disablePMOverride();
	void setPM(bool en);		// Requires enablePMOverride to be in effect.

	void softReset();

	void setThresholds(Thresholds thresholds);
	void setOffsets(Offsets offsets);

	private:
	I2CAddr i2cAddr;
	float vInGain, vOutGain, iInGain, iOutGain;
	bool adcOverride = false;

	void setADCOverrideMaxOutVI(float viOutMax, float gain, uint8_t offset, uint64_t mask);

	template<class T> T get(RegisterAddr regAddr);
	template<class T> void set(RegisterAddr regAddr, T reg);

	static bool isWritableRegister(RegisterAddr reg);

	static SM72445::Status I2C_WriteBits(I2CAddr i2cAddr, RegisterAddr regAddr, uint64_t bits, uint64_t mask);
	static SM72445::Status I2C_SetBits(I2CAddr i2cAddr, RegisterAddr regAddr, uint64_t bits);
	static SM72445::Status I2C_ResetBits(I2CAddr i2cAddr, RegisterAddr regAddr, uint64_t bits);

	/**
	 * @brief Template function that can be used to modify a register based on the .
	 * 
	 * @param i2cAddr	The address of the SM72445 to act upon.
	 * @param regAddr	The address of the SM72445 internal register to act upon.
	 * @param bits		Mask of bits that are to be set or reset.
	 * @param setReset  
	 * @return SM72445::Status 
	 */
	template<typename T>
	static uint64_t I2C_Mutate(I2CAddr i2cAddr, RegisterAddr regAddr, uint64_t bits, T&& action, uint64_t mask = 0x0u);

	/*************************************************************************************************************************************
	 * System-level I2C communication methods. To be implemented by host applciaiton. */

	/**
	 * @brief Read a single memory register from the SM72445, given a I2C address.
	 * 
	 * @param i2cAddr	The address of the SM72445 to read from.
	 * @param regAddr	The address of the SM72445 internal register to read from.
	 * @param pData		Pointer to a 64-bit unsigned integer to save data to.
	 * @return SM72445::Status SM72445::Status::OK if success, else SM72445::Status::FAIL_I2C.
	 *  
	 * @note Set all unused bits of pData to 0.
	 * @note The SM72445 registers are all 7 bytes wide (SM72445_REG_SIZE).
	 * @note All bytes must be read in a single sequence.
	 * @note Refer to Page 14 of the device datasheet for more information.
	 */
	static SM72445::Status I2C_MemRead (uint8_t i2cAddr, uint8_t regAddr, uint64_t & rData);

	/**
	 * @brief Write a single memory register from A SM72445, given an I2C address.
	 * 
	 * @param i2cAddr	The address of the SM72445 to write to.
	 * @param regAddr	The address of the SM72445 internal register to write to.
	 * @param pData		Pointer to a 64-bit unsigned integer to save data to.
	 * @return SM72445::Status SM72445::Status::OK if success, else SM72445::Status::FAIL_I2C.
	 * 
	 * @note Set all unused bits of pData to 0.
	 * @note The SM72445 registers are all 7 bytes wide (SM72445_REG_SIZE).
	 * @note All bytes must be written in a single sequence.
	 * @note Refer to Page 14 of the device datasheet for more information.
	 */
	static SM72445::Status I2C_MemWrite(uint8_t i2cAddr, const uint8_t regAddr, uint64_t data);

	/*************************************************************************************************************************************/

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
	//REG2 = 0xE2u,	//Not a visible register
	REG3 = 0xE3u,
	REG4 = 0xE4u,
	REG5 = 0xE5u
};

enum class FreqPanelMode : uint8_t{
	//F_HGH_PM_SWITCH = 0x00u,	// Unsupported. Use alternative option.
	F_MED_PM_SWITCH = 0x01u,
	F_LOW_PM_SWITCH = 0x02u,
	F_HGH_PM_BRIDGE = 0x03u,
	F_MED_PM_BRIDGE = 0x04u,
	F_LOW_PM_BRIDGE = 0x05u,
	F_HGH_PM_SWITCH = 0x07u	
};

enum class SM72445::Status : int{
	OK = 0,
	FAIL_I2C = -1
};

template <typename T>
struct SM72445::QuadDataStruct{
	protected:
	uint8_t bitsize;
	T data[4];

	public:
	QuadDataStruct();
	QuadDataStruct(const uint8_t bitsize, T u1, T u2, T u3, T u4);
	QuadDataStruct(const uint8_t bitsize, uint64_t reg);

	uint64_t toReg();

	QuadDataStruct & operator = (const QuadDataStruct & qds) = default;
};

template <typename T>
struct SM72445::VoltsCurrents : QuadDataStruct<T>{
	T & vOut { this->data[3] };
	T & iOut { this->data[2] };
	T & vIn  { this->data[1] };
	T & iIn  { this->data[0] };

	VoltsCurrents();
	VoltsCurrents(const uint8_t bitsize, T vOut, T iOut, T vIn, T iIn);
	VoltsCurrents(const uint8_t bitsize, uint64_t reg);

	VoltsCurrents & operator = (const VoltsCurrents & vc);
};

struct SM72445::ADCs : QuadDataStruct<uint16_t>{
	uint16_t & ADC6 { this->data[3] };
	uint16_t & ADC4 { this->data[2] };
	uint16_t & ADC2 { this->data[1] };
	uint16_t & ADC0 { this->data[0] };

	ADCs();
	ADCs(uint64_t reg);

	ADCs & operator = (const ADCs & adc);
};

struct SM72445::Sensors : VoltsCurrents<uint16_t>{
	Sensors();
	Sensors(uint64_t reg);

	Sensors & operator = (const Sensors & sensor);
};

struct SM72445::Offsets : VoltsCurrents<uint8_t>{
	Offsets();
	Offsets(uint8_t vOut, uint8_t iOut, uint8_t vIn, uint8_t iIn);
	Offsets(uint64_t reg);

	Offsets & operator = (const Offsets & offsets);
};

struct SM72445::Thresholds : QuadDataStruct<uint16_t>{
	uint16_t & iInHigh  { this->data[3] };
	uint16_t & iInLow   { this->data[2] };
	uint16_t & iOutHigh { this->data[1] };
	uint16_t & iOutLow  { this->data[0] };

	Thresholds();
	Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow);
	Thresholds(uint64_t reg);

	Thresholds & operator = (const Thresholds & th);
};
/* End Public Class Definitions */

#endif /* INC_SM72245_HPP_ */

/*** END OF FILE ***/

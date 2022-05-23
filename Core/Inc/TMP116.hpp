/**
  ******************************************************************************
  * @file           : TMP116.hpp
  * @brief          : Header file for TMP116.cpp
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

#pragma once

/* --------------------------------------------------------------------------- */
/* Begin Public Includes */
#include <stdint.h>
#include <string>

/* End Public Includes */

/* --------------------------------------------------------------------------- */
/* Begin Public Defines */
#define TMP116_REG_SIZE   2


/**
 * @defgroup Register Reset Values
 * @note Items designated with MFG_RST may have user defined reset values programmed
 * 		 in the EEPROM.
 */
#define TMP116_TEMP_PORV 			0x8000u
#define TMP116_CFGR_MFG_RST 		0x0220u
#define TMP116_HIGH_LIM_MFG_RST 	0x6000u
#define TMP116_LOW_LIM_MFG_RST 		0x8000u
#define TMP116_EEPROM_UL_PORV	 	0x0000u
#define TMP116_EEPROM1_MFG_RST 		0x0000u
#define TMP116_EEPROM2_MFG_RST 		0x0000u
#define TMP116_EEPROM3_MFG_RST 		0x0000u
#define TMP116_EEPROM4_MFG_RST 		0x0000u
#define TMP116_DEV_ID 				0x1116u

/* End Public Defines */

/* --------------------------------------------------------------------------- */
/* Begin Public Typedefs */


/* End Public Typedefs */

/* --------------------------------------------------------------------------- */
/* Begin Global Public Variables */


/* End Global Public Variables */

/* --------------------------------------------------------------------------- */
/* Begin Public Function Prototypes */


/* End Public Function Prototypes */

/* --------------------------------------------------------------------------- */
/* Begin Public Class Definitions */
class TMP116{
public:
	/* Enumerators for various parameters. */
	enum class I2CAddr : uint8_t;
	enum class ConversionMode : uint16_t;
	enum class ConversionTime : uint16_t;
	enum class Averages : uint16_t;
	enum class ThermAlertMode : uint16_t;
	enum class Polarity : uint16_t;
	enum class AlertPinMode : uint16_t;

	/* Constructors */
	TMP116(I2CAddr i2cAddr, std::string proximity = "");

	/* Getters */
	int16_t getTemperatureReg();
	float getTemperature();
	uint16_t getDevID();
	
	bool getHighAlert();
	bool getLowAlert();
	bool dataReady();
	bool eepromBusy();

	/* Setters */
	void setConversionMode(ConversionMode mode = ConversionMode::CONTINUOUS);
	void setConversionCycle(ConversionTime conv);
	void setAverages(Averages avg);
	void setAlertMode(ThermAlertMode mode);
	void setAlertPolarity(Polarity pol);
	void setAlertPinMode(AlertPinMode mode);

	void setHighLimit(int16_t limit);
	void setHighLimit(float limit);
	void setLowLimit(int16_t limit);
	void setLowLimit(float limit);

private:
	/* Identifiers */
	I2CAddr i2cAddr;
	std::string proximity;

	/* Internal Enumerators */
	enum class RegisterAddr : uint8_t;
	enum class Status : int;

	/* Helper Methods */
	bool getConfigTrue(uint16_t mask);

	template<typename T>
	static uint16_t I2C_Mutate(I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t bits, T&& action, uint16_t mask);

	/*************************************************************************************************************************************
	 * System-level I2C communication methods. To be implemented by host application. */

	/**
	 * @brief Read a single memory register from the TMP116, given a I2C address.
	 * 
	 * @param i2cAddr	The address of the TMP116 to read from.
	 * @param regAddr	The address of the TMP116 internal register to read from.
	 * @param rData		Reference to a 16-bit unsigned integer to save data to.
	 * @return TMP116::Status TMP116::Status::OK if success, else TMP116::Status::FAIL_I2C.
	 *  
	 * @note All TMP116 registers are all 2 bytes wide (TMP116_REG_SIZE).
	 */
	static Status I2C_MemRead (I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t & rData);

	/**
	 * @brief Write a single memory register from a TMP116, given an I2C address.
	 * 
	 * @param i2cAddr	The address of the TMP116 to write to.
	 * @param regAddr	The address of the TMP116 internal register to write to.
	 * @param data		Pointer to a 16-bit unsigned integer to save data to.
	 * @return TMP116::Status TMP116::Status::OK if success, else TMP116::Status::FAIL_I2C.
	 * 
	 * @note The TMP116 registers are all 2 bytes wide (TMP116_REG_SIZE).
	 */
	static Status I2C_MemWrite(I2CAddr i2cAddr, RegisterAddr regAddr, uint16_t data);

	/*************************************************************************************************************************************/

	/* Enum Class Definitions */
private:

	/**
	 * @brief TMP116 Register Map
	 * @note  All registers are R/W except those expressly noted below.
	 */
	enum class RegisterAddr : uint8_t{
		TEMP 		= 0x00u,	/* Temperature register */		/* READ ONLY */
		CFGR 		= 0x01u,	/* Configuration Register */
		HIGH_LIM 	= 0x02u,	/* High Limit Register */
		LOW_LIM 	= 0x03u,	/* Low Limit Register */
		EEPROM_UL 	= 0x04u,	/* EEPROM Unlock Register */
		EEPROM1 	= 0x05u,	/* EEPROM Register 1 */
		EEPROM2 	= 0x06u,	/* EEPROM Register 2 */
		EEPROM3 	= 0x07u,	/* EEPROM Register 3 */
		EEPROM4 	= 0x08u,	/* EEPROM Register 4 */
		DEVICE_ID 	= 0x0Fu 	/* Device ID Register */		/* READ ONLY */
	};

	/**
	 * @brief Internal method status return type / error code enumerable.
	 */
	enum class Status : int{
		OK = 0,
		FAIL_I2C = -1,
		FAIL_INVALID_PARAMETER = -2
	};

public:
	/**
	 * @brief TMP116 I2C Addresses Enumerable.
	 */
	enum class I2CAddr : uint8_t{
		ADDR0 = 0b1001000u,	/* ADD0 connected to GND */
		ADDR1 = 0b1001001u,	/* ADD0 connected to VCC */
		ADDR2 = 0b1001010u,	/* ADD0 connected to SDA */
		ADDR3 = 0b1001011u 	/* ADD0 connected to SCL */
	};

	/**
	 * @brief TMP116 Conversion Modes Enumerable.
	 */
	enum class ConversionMode : uint16_t{
		CONTINUOUS 	= 0b00 << 10,
		SHUTDOWN 	= 0b01 << 10,
		ONE_SHOT	= 0b11 << 10
	};

	/**
	 * @brief TMP116 Conversion Cycle Times Enumerable.
	 * @note At conversion cycle times less than 1000ms, if a high number of averaging
	 * 		 measurements are programmed, the conversion time will increase beyond this
	 * 		 programmed value.
	 * 		 Consult Table 7 on Page 27 of the Datasheet for exact information.
	 */
	enum class ConversionTime : uint16_t{
		MS0015  = 0b000 << 7,
		MS0125  = 0b001 << 7,
		MS0250  = 0b010 << 7,
		MS0500  = 0b011 << 7,
		MS1000  = 0b100 << 7,
		MS4000  = 0b101 << 7,
		MS8000  = 0b110 << 7,
		MS16000 = 0b111 << 7,
		MAX = MS16000,
		MIN = MS0015
	};

	/**
	 * @brief Enumerable for number of measurements to average.
	 * @note Programming a high number of averages at short conversion cycles may force the 
	 * 		 cycle time to increase beyond the programmed value.
	 * 		 Consult Table 7 on Page 27 of the Datasheet for exact information.
	 */
	enum class Averages : uint16_t{
		AVG01 = 0b00 << 5,
		AVG08 = 0b01 << 5,
		AVG32 = 0b10 << 5,
		AVG64 = 0b11 << 5,
		SINGLE = AVG01,
		MAX = AVG64
	};

	/**
	 * @brief Enumerable for Thermal Alert Modes.
	 */
	enum class ThermAlertMode : uint16_t{
		THERM = 0b1u << 4,
		ALERT = 0
	};

	/**
	 * @brief Enumerable for the Polarity of the TMP116 Alert Pin.
	 */
	enum class Polarity : uint16_t{
		ACTIVE_HIGH = 0b1u << 3,
		ACTIVE_LOW = 0
	};

	/**
	 * @brief Enumerable for Modes of the TMP116 Alert Pin.
	 */
	enum class AlertPinMode : uint16_t{
		DATA_READY = 0b1u << 2,
		ALERT_FLAGS = 0
	};
	
}; /* TMP116 */




/* End Public Class Definitions */

/*** END OF FILE ***/

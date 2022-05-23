/**
  ******************************************************************************
  * @file           : SISPSPV.hpp
  * @brief          : Header file for SISPSPV.cpp
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

/**
 * @note This driver assumes a STM32 Hardware Abstraction Layer environment and
 * various automatically-generated definitions in main.hpp.
 */


#pragma once

/* --------------------------------------------------------------------------- */
/* Begin Public Includes */
#include <stdint.h>
#include <string>
#include "main.hpp"
#include "SM72445.hpp"
#include "HDC1080.hpp"
#include "TMP116.hpp"

using string  = std::string;
/* End Public Includes */

/* --------------------------------------------------------------------------- */
/* Begin Public Defines */
#define SISPSPV_BOARD_REVISION 0

#define SM72445_VIN_GAIN  0.105969f
#define SM72445_VOUT_GAIN 0.199029f
#define SM72445_IIN_GAIN  0.8f
#define SM72445_IOUT_GAIN 0.8f

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
class SISPSPV{
public:
	struct GPIO;

	class MPPT;
	class eFuse;
	class EEPROM;
	class Backplane;
	class TempSensors;
	template <typename Handle> class UART;
	typedef HDC1080 HumdSensor;

	SISPSPV();

	void writeLED(uint8_t n);

private:
	static MPPT mppt;
	static eFuse efuse;
	static Backplane backplane;
	static EEPROM eeprom;
	static HumdSensor hum;
	static TempSensors tmp;

	static const GPIO resetOut;

	static const GPIO LED1;
	static const GPIO LED2;
	static const GPIO LED3;
	static const GPIO LEDs;
	
	static inline void delay(uint32_t ms){HAL_Delay(ms);}
};

struct SISPSPV::GPIO{
	GPIO_TypeDef * port;
	uint16_t pin;

	constexpr GPIO(GPIO_TypeDef * port, uint16_t pin);

	static inline void set(GPIO gpio, GPIO_PinState pinState){HAL_GPIO_WritePin(gpio.port, gpio.pin, pinState);}
	static inline void toggle(GPIO gpio){HAL_GPIO_TogglePin(gpio.port, gpio.pin);}
	static inline GPIO_PinState read(GPIO gpio){return HAL_GPIO_ReadPin(gpio.port, gpio.pin);}
};

class SISPSPV::MPPT : SM72445{
public:
	void forcePanelMode();
	bool powerGood();
	
private:
	static const GPIO pmForce;
	static const GPIO pGood;
};

template<typename Handle>
class SISPSPV::UART{
protected:

	Handle h;

public:
	UART(Handle handle);

	void print(string str);
	string scan();
	void transmit(string str);
	string receive();
};

class SISPSPV::eFuse{
public:
	bool fuseOK();
	bool fuseOverCurrent();

private:
	static const GPIO OK;
	static const GPIO OC;
};

class SISPSPV::Backplane{
public:


private:
	static const GPIO stop;
	static const GPIO wake;
};

class SISPSPV::EEPROM{
public:

private:
	static const GPIO wc;
};

class SISPSPV::TempSensors {
public:
	TempSensors();
	

private:
	static const TMP116 tmp1;
	static const TMP116 tmp2;
	static const TMP116 tmp3;
	static const TMP116 tmp4;
	static const GPIO smba;
};

/* End Public Class Definitions */

/*** END OF FILE ***/

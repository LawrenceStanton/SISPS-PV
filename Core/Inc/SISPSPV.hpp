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
 * @note This driver assumes a STM32 Hardware Abstraction Layer environment.
 */


#ifndef INC_SISPSPV_HPP_
#define INC_SISPSPV_HPP_

/* --------------------------------------------------------------------------- */
/* Begin Public Includes */
#pragma once
#include <stdio.h>
#include "main.h"
#include "SM72445.hpp"
#include "HDC1080.hpp"
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
	struct GPIO{
		GPIO_TypeDef * port;
		uint16_t pin;

		constexpr GPIO(GPIO_TypeDef * port, uint16_t pin);
	};

	class MPPT : SM72445{
		public:
		void forcePanelMode();
		bool powerGood();

		
		private:
		static const GPIO pmForce;
		static const GPIO pGood;
		
	};

	class eFuse{
		public:
		bool fuseOK();
		bool fuseOverCurrent();

		private:
		static const GPIO fuseOK;
		static const GPIO fuseOC;
	};

	class EEPROM{
		private:
		static const GPIO wc;

	};

	class Backplane{
		private:
		static const GPIO stop;
		static const GPIO wake;

	};

	SISPSPV();

	void writeLED(uint8_t n);

	private:
	static MPPT mppt;
	static eFuse efuse;
	static EEPROM eeprom;
	static Backplane backplane;
	static HDC1080 hdc;

	static const GPIO resetOut;

	static const GPIO LED1;
	static const GPIO LED2;
	static const GPIO LED3;
	
	static inline void delay(uint32_t ms){HAL_Delay(ms);}
	static inline void setGPIO(GPIO gpio, GPIO_PinState pinState){HAL_GPIO_WritePin(gpio.port, gpio.pin, pinState);}
	static inline void toggleGPIO(GPIO gpio){HAL_GPIO_TogglePin(gpio.port, gpio.pin);}
	static inline GPIO_PinState readGPIO(GPIO gpio){return HAL_GPIO_ReadPin(gpio.port, gpio.pin);}
};


/* End Public Class Definitions */

#endif /* INC_SISPSPV_HPP_ */

/*** END OF FILE ***/

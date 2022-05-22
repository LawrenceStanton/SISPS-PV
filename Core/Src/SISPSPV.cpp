/**
  ******************************************************************************
  * @file           : SISPSPV.cpp
  * @brief          : Source / Header file for SISPSPV.c/.h
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
#include "SISPSPV.hpp"

/* End Private Includes */

/* --------------------------------------------------------------------------- */
/* Begin Private Defines */


/* End Private Defines */

/* --------------------------------------------------------------------------- */
/* Begin Private Macros */


/* End Private Macros */

/* --------------------------------------------------------------------------- */
/* Begin Private Typedefs */


/* End Private Typedefs */

/* --------------------------------------------------------------------------- */
/* Begin Private Function Prototypes */


/* End Private Function Prototypes */

/* --------------------------------------------------------------------------- */
/* Begin Private Variables */
const SISPSPV::GPIO SISPSPV::MPPT::pmForce 	 = GPIO(PM_FORCE_GPIO_Port,  PM_FORCE_Pin);
const SISPSPV::GPIO SISPSPV::MPPT::pGood 	 = GPIO(PGOOD_GPIO_Port, 	 PGOOD_Pin);
const SISPSPV::GPIO SISPSPV::eFuse::OK	 	 = GPIO(FUSE_OK_GPIO_Port, 	 FUSE_OK_Pin);
const SISPSPV::GPIO SISPSPV::eFuse::OC	 	 = GPIO(FUSE_OC_GPIO_Port, 	 FUSE_OC_Pin);
const SISPSPV::GPIO SISPSPV::EEPROM::wc		 = GPIO(EEPROM_WC_GPIO_Port, EEPROM_WC_Pin);
const SISPSPV::GPIO SISPSPV::Backplane::stop = GPIO(STOP_GPIO_Port, 	 STOP_Pin);
const SISPSPV::GPIO SISPSPV::Backplane::wake = GPIO(WAKE_GPIO_Port, 	 WAKE_Pin);
const SISPSPV::GPIO SISPSPV::resetOut		 = GPIO(RST_OUT_GPIO_Port, 	 RST_OUT_Pin);
const SISPSPV::GPIO SISPSPV::LED1			 = GPIO(LED1_GPIO_Port, 	 LED1_Pin);
const SISPSPV::GPIO SISPSPV::LED2			 = GPIO(LED2_GPIO_Port, 	 LED2_Pin); 
const SISPSPV::GPIO SISPSPV::LED3			 = GPIO(LED3_GPIO_Port, 	 LED3_Pin);

const SISPSPV::GPIO SISPSPV::LEDs			 = GPIO(LED1_GPIO_Port, 	 LED3_Pin);

/* End Private Variables */

/* --------------------------------------------------------------------------- */
/* Begin Source Code */

constexpr SISPSPV::GPIO::GPIO(GPIO_TypeDef * port, uint16_t pin):
	port(port), pin(pin){}

void SISPSPV::writeLED(uint8_t n){
	if(n > 7) n = 0;
	
	GPIO::set(LED1, ((n >> 0) & 0b1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	GPIO::set(LED1, ((n >> 1) & 0b1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	GPIO::set(LED1, ((n >> 2) & 0b1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific GPIO Implementations
 * For convenience, the below methods are methods are suitable implementations of the I2C communication methods for
 * STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL).
 *************************************************************************************************************************/
#define SISPS_USE_STM32_HAL_METHODS
#ifdef SISPS_USE_STM32_HAL_METHODS

#include "stm32g0xx_hal.h"	// STM32 HAL driver to be included. Adjust depending on the specific STM32 platform.




#endif /* SISPS_USE_STM32_HAL_METHODS */
/* End Source Code */


/*** END OF FILE ***/

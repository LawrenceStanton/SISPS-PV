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

#ifndef INC_SISPSPV_HPP_
#define INC_SISPSPV_HPP_

/* --------------------------------------------------------------------------- */
/* Begin Public Includes */
#pragma once
#include <stdio.h>
#include "SM72445.hpp"
#include "HDC1080.hpp"
/* End Public Includes */

/* --------------------------------------------------------------------------- */
/* Begin Public Defines */


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
class MPPT : SM72445{
	public:



	private:
	/*************************************************************************************************************************************
	 * System-level GPIO control methods. To be implemented by host application. */
	template <typename PortType, typename PinType>
	void setGPIO(PortType port, PinType pin);

	template <typename PortType, typename PinType>
	void toggleGPIO(PortType port, PinType pin);
	
	template <typename StateType, typename PortType, typename PinType>
	StateType readGPIO(PortType port, PinType pin);
};


/* End Public Class Definitions */

#endif /* INC_SISPSPV_HPP_ */

/*** END OF FILE ***/

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
 * @brief Lambda functions to be used with I2C_Mutate.
 */
static const auto lambda_set   = [](uint64_t reg, uint64_t bits, uint64_t) -> uint64_t { return reg |  bits; };
static const auto lambda_reset = [](uint64_t reg, uint64_t bits, uint64_t) -> uint64_t { return reg & ~bits; };
static const auto lambda_write = [](uint64_t reg, uint64_t bits, uint64_t mask) -> uint64_t { return (reg & ~mask) | (mask & bits); };

/* End Private Variables */

/* Begin Source Code */

/**
 * @brief Construct a new SM72445::SM72445 object.
 * 
 * @param i2cAddr	The I2C Address of the SM72445.
 * @param vInGain	AVIN 	/ vIn
 * @param vOutGain	AVOUT 	/ vOut
 * @param iInGain	AIIN 	/ iIn
 * @param iOutGain	AIOUT 	/ iOut
 * 
 * @note AVIN, AVOUT, AIIN, AIOUT refer to the voltages on those respective nets as identified by the SM72445 datasheet.
 */
SM72445::SM72445(I2CAddr i2cAddr, float vInGain, float vOutGain, float iInGain, float iOutGain ){
	this->i2cAddr = i2cAddr;
	this->vInGain = vInGain;
	this->vOutGain = vOutGain;
	this->iInGain = iInGain;
	this->iOutGain = iOutGain;
}

/**
 * @brief Gets the values of the SM72445 ADC register (REG0).
 * 
 * @return SM72445::ADCs
 * 
 * @note A return containing all zeros may indicate an I2C communication failure.
 */
SM72445::ADCs SM72445::getADC_Values(){
	return get<ADCs>(RegisterAddr::REG0);
}

/**
 * @brief Gets the values of the SM72445 Sensors register (REG1).
 * 
 * @return SM72445::Sensors 
 * 
 * @note A return containing all zeros may indicate an I2C communication failure.
 */
SM72445::Sensors SM72445::getSensors(){
	return get<Sensors>(RegisterAddr::REG1);
}

/**
 * @brief Gets the values of the SM72445 Offsets register (REG4).
 * 
 * @return SM72445::Offsets 
 * 
 * @note A return containing all zeros may indicate an I2C communication failure.
 */
SM72445::Offsets SM72445::getOffsets(){
	return get<Offsets>(RegisterAddr::REG4);
}

/**
 * @brief Gets the values of the SM72445 Thresholds register (REG5).
 * 
 * @return SM72445::Thresholds 
 * 
 * @note A return containing all zeros may indicate an I2C communication failure.
 */
SM72445::Thresholds SM72445::getThresholds(){
	return get<Thresholds>(RegisterAddr::REG5);
}

/**
 * @brief General purpose getter method for SM72445.
 * 
 * @tparam T The data structure to submit register data to. Should be either ADCs, Sensors, Offsets or Thresholds.
 * @param regAddr	The SM72445 register address to fetch data from.
 * @return T The data structure constructed with the retched register data.
 */
template<class T>
T SM72445::get(RegisterAddr regAddr){
	uint64_t reg = 0x0ull;
	I2C_MemRead((uint8_t)this->i2cAddr, (uint8_t)regAddr, reg);

	return T(reg);
}

/**
 * @brief Enables Switching Frequency and Panel Mode Configuration ADC Overriding.
 * 
 * @param fpm	Enumerable for Frequency and Panel Mode setting to override with.
 * 
 * @note This method follows the procedure described on Page 13 of the SM72445 Datasheet, as "quoted".
 */
void SM72445::setADCOverrideFreqPM(FreqPanelMode fpm){
	uint64_t reg;

	if(!this->adcOverride){	// If override has not beeen done before.
		this->adcOverride = true;	// SM72445 ADC registers are now to be overridden.

		// "Set the bb_reset bit, reset the over-ride bit, set the desired PWM code."
		reg = I2C_Mutate(
			this->i2cAddr,
			RegisterAddr::REG3,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | ( ((uint64_t)fpm) << SM72445_CONFIG_A2_OVERRIDE_OFFSET),
			lambda_write,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | SM72445_CONFIG_A2_OVERRIDE_MASK);
		
		if(reg != 0x0ull){	// 0x0ull result indicates communication failure.

			// "Reset the bb_reset bit, keep the over-ride bit, keep the desired PWM code."
			reg = reg & ~SM72445_CONFIG_RESET_BIT;
			I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg);
		}
	}
	else{	// Override has been done before.

		// "Set bb_reset bit, reset over-ride bit, set the desired PWM code.""
		reg = I2C_Mutate(
			this->i2cAddr,
			RegisterAddr::REG3,
			SM72445_CONFIG_RESET_BIT | ( ((uint64_t)fpm) << SM72445_CONFIG_A2_OVERRIDE_OFFSET),
			lambda_write,
			SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT | SM72445_CONFIG_A2_OVERRIDE_MASK);

		if(reg != 0x0ull){	// 0x0ull result indicates communication failure.
			
			// "Reset the bb_reset bit, reset the over-ride bit, keep the desired PWM code."
			reg = reg & ~(SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT);
			if(I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg) == Status::OK){

				// "Set the bb_reset bit, set the over-ride bit, keep he desired PWM code."
				reg = reg | SM72445_CONFIG_RESET_BIT | SM72445_CONFIG_ADC_OVERRIDE_BIT;
				if(I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg) == Status::OK){
				
					// "Reset the bb_reset bit, keep the over-ride bit, keep the desired PWM code.""
					reg = reg & ~SM72445_CONFIG_RESET_BIT;
					I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg);
				}
			}
		}
	}
}

/**
 * @brief Sets the Maximum Outout Current for an ADC override.
 * 
 * @param iOutMax	The maximum output current in Amps.
 * 
 * @note The SM72445 requires the ADC Frequency and Panel Mode setting to also be overridden in order for this override to take effect.
 */
void SM72445::setADCOverrideMaxOutI(float iOutMax){
	setADCOverrideMaxOut(iOutMax, this->iOutGain, SM72445_CONFIG_IOUTMAX_OVERRIDE_OFFSET, SM72445_CONFIG_IOUTMAX_OVERRIDE_MASK);
}

/**
 * @brief Sets the Maximum Output Voltage for an ADC override.
 * 
 * @param vOutMax	The maximum output voltage in Volts.
 * 
 * @note The SM72445 requires the ADC Frequency and Panel Mode setting to also be overridden in order for this override to take effect.
 */
void SM72445::setADCOverrideMaxOutV(float vOutMax){
	setADCOverrideMaxOut(vOutMax, this->vOutGain, SM72445_CONFIG_VOUTMAX_OVERRIDE_OFFSET, SM72445_CONFIG_VOUTMAX_OVERRIDE_MASK);
}

/**
 * @brief General purpose method for setting maximum output voltage current values.
 * 
 * @param viOutMax	The maximum output voltage or current in Volts or Amps respectfully.
 * @param gain		The respective gain of the value passed in viOutMax.
 * @param offset	The offset of the value passed in viOutMax within REG3 of the SM72445.
 * @param mask		The mask of the value passed in viOutMax within REG3 of the SM72445.
 */
void SM72445::setADCOverrideMaxOut(float viOutMax, float gain, uint8_t offset, uint64_t mask){
	float viOutMaxADC = viOutMax * gain;

	if((viOutMax >= 0.0) && (viOutMaxADC < SM72445_VDDA)){
		uint16_t viOutMaxD = (uint16_t) (SM72445_ADC_FS_RANGE * (viOutMaxADC / SM72445_VDDA) );
	
		I2C_Mutate(
			this->i2cAddr, 
			RegisterAddr::REG3,
			(((uint64_t)viOutMaxD) << offset), 
			lambda_write,
			mask);
	}
}

/**
 * @brief Short method for seting both the maximum output voltage and maximum output current ADC override values simultaniously.
 * 
 * @param vOutMax	The maximum output voltage in Volts.
 * @param iOutMax	The maximum output current in Amps.
 * 
 * @note The SM72445 requires the ADC Frequency and Panel Mode setting to also be overridden in order for this override to take effect.
 */
void SM72445::setADCOverrideMaxOutVI(float vOutMax, float iOutMax){
	const float vOutMaxADC = vOutMax * this->vOutGain;
	const float iOutMaxADC = iOutMax * this->iOutGain;

	if((vOutMax >= 0.0) && (vOutMaxADC < SM72445_VDDA) && (iOutMax >= 0.0) && (iOutMaxADC < SM72445_VDDA)){
		this->adcOverride = true;	// SM72445 ADC registers are now to be overridden.

		uint16_t vOutMaxD = (uint16_t) (SM72445_ADC_FS_RANGE * (vOutMaxADC / SM72445_VDDA) );
		uint16_t iOutMaxD = (uint16_t) (SM72445_ADC_FS_RANGE * (iOutMaxADC / SM72445_VDDA) );

		I2C_Mutate(
			this->i2cAddr,
			RegisterAddr::REG3,
				( ((uint64_t)vOutMaxD) << SM72445_CONFIG_VOUTMAX_OVERRIDE_OFFSET ) | 
				( ((uint64_t)iOutMaxD) << SM72445_CONFIG_IOUTMAX_OVERRIDE_OFFSET ) ,
			lambda_write,
			SM72445_CONFIG_VOUTMAX_OVERRIDE_MASK | SM72445_CONFIG_IOUTMAX_OVERRIDE_MASK );
	}
}

/**
 * @brief Enables overriding of the Panel Mode output.
 */
void SM72445::enablePMOverride(){
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_OVERRIDE_BIT, lambda_set);
}

/**
 * @brief Disables overriding of the Panel Mode output.
 */
void SM72445::disablePMOverride(){
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_OVERRIDE_BIT, lambda_reset);
}

/**
 * @brief Sets the state of the panel mode override.
 * 
 * @param en true to enable Panel Mode, false to disable Panel Mode
 * 
 * @note The SM72445 requires the Panel Mode Override bit to be set in order for this override to have effect (enablePMOverride()).
 */
void SM72445::setPM(bool en){
	auto action = en ? lambda_set : lambda_reset;
	I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_PM_SET_BIT, action);
}

/**
 * @brief Performs a soft reset via I2C of the SM72445.
 * 
 * @note This method follows the procedure described on page 13 of the SM72445 datasheet.
 */
void SM72445::softReset(){
	uint64_t reg = I2C_Mutate(this->i2cAddr, RegisterAddr::REG3, SM72445_CONFIG_RESET_BIT, lambda_set);
	if(reg != 0u) {
		reg &= ~SM72445_CONFIG_RESET_BIT;
		I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)RegisterAddr::REG3, reg);
	}
}

/**
 * @brief Sets the offset values within the SM72445 REG4.
 * 
 * @param offsets Offsets for Input Voltage, Input Current, Output Voltage, and Output Current.
 * 
 * @note These values are directly added or subtracted from the results of SM72445 ADC and will affect the measured Sensor and Threshold values.
 */
void SM72445::setOffsets(Offsets offsets){
	set(RegisterAddr::REG4, offsets);
}

/**
 * @brief Sets the threshold values within the SM72445 REG5.
 * 
 * @param thresholds Threshold values for Input Current High, Input Current Low, Output Current High, and Output Current Low.
 */
void SM72445::setThresholds(Thresholds thresholds){
	set(RegisterAddr::REG5, thresholds);
}

/**
 * @brief General purpose method for setting REG0, REG2, REG4 or REG5 with values in the form derived from QuadDataStruct.
 * 
 * @tparam T The applicable data structure derived from QuadDataStruct.
 * @param regAddr	The address to crite to.
 * @param reg		The structure T containing the data to write.
 */
template<class T>
void SM72445::set(RegisterAddr regAddr, T reg){
	I2C_MemWrite((uint8_t)this->i2cAddr, (uint8_t)regAddr, reg.toReg() );
}

/**
 * @brief Construct a new SM72445::QuadDataStruct<T>::QuadDataStruct object
 * 
 * @tparam T 
 */
template<typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct() :
	SM72445::QuadDataStruct<T>::QuadDataStruct(RegisterAddr::REG0, 0, 0, 0, 0) {}

/**
 * @brief Construct a new SM72445::QuadDataStruct<T>::QuadDataStruct object.
 * 
 * @tparam T The minimum-sized data type.
 * @param bitsize The register bitsize (which is less than or equal to the bit size of type T).
 * @param u1 The lest significant data variable as stored in the SM72445 register.
 * @param u2 The second lest significant data variable as stored in the SM72445 register.
 * @param u3 The second most significant data variable as stored in the SM72445 register.
 * @param u4 The most significant data variable as stored in the SM72445 register.
 */
template <typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct(const uint8_t bitsize, T u1, T u2, T u3, T u4){
	this->bitsize = bitsize;

	if((u1 >> bitsize) || (u2 >> bitsize) || (u3 >> bitsize) || (u4 >> bitsize)){
		this->data[0] = (T)0x0;
		this->data[1] = (T)0x0;
		this->data[2] = (T)0x0;
		this->data[3] = (T)0x0;	
	}
	else{
		this->data[0] = u1;
		this->data[1] = u2;
		this->data[2] = u3;
		this->data[3] = u4;
	}
}

/**
 * @brief Construct a new SM72445::QuadDataStruct<T>::QuadDataStruct object
 * 
 * @tparam T The minimum-sized variable data type.
 * @param bitsize The actual bitsize of the variable as stored in the SM72445.
 * @param reg
 */
template <typename T>
SM72445::QuadDataStruct<T>::QuadDataStruct(const uint8_t bitsize, const uint64_t reg){
	this->bitsize = bitsize;
	
	const T mask = (0b1u << bitsize) - 1;

	for(int i = 0; i < 4; i++) this->data[i] = ( reg >> (bitsize*i) ) & mask;
}

/**
 * @brief Formats a QuadDataStruct into a single variable matching the register format within the SM72445.
 * 
 * @tparam T The base data type of each variable being formated.
 * @return uint64_t	The formatted register variable.
 * 
 * @note All unused bits are set to 0.
 * @note This method checks to ensure that each and every variable has respected the bitsize, else returns 0x0ull.
 */
template<typename T>
uint64_t SM72445::QuadDataStruct<T>::toReg(){
	uint64_t reg = 0x0ull;

	if( !( (this->data[0] >> this->bitsize) || (this->data[1] >> this->bitsize) || (this->data[2] >> this->bitsize) || (this->data[3] >> this->bitsize) ) )
	for(int i = 3; i >= 0; i--) reg = (reg << this->bitsize) | this->data[i];

	return reg;
}

/**
 * @brief Construct a new SM72445::VoltsCurrents<T>::VoltsCurrents object.
 * 
 * @tparam T The base variable minimum-size data type.
 * 
 * @note Default constructor.
 */
template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents() : 
	SM72445::VoltsCurrents<T>::VoltsCurrents(0, 0, 0, 0, 0) {}

/**
 * @brief Construct a new SM72445::VoltsCurrents<T>::VoltsCurrents object
 * 
 * @tparam T The base variable minimum-size data type.
 * @param bitsize The variable bitsize within the SM72445 register.
 * @param vOut
 * @param iOut 
 * @param vIn
 * @param iIn
 */
template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents(const uint8_t bitsize, T vOut, T iOut, T vIn, T iIn) : 
	SM72445::QuadDataStruct<T>::QuadDataStruct(bitsize, vOut, iOut, vIn, iIn) {}

/**
 * @brief Construct a new SM72445::VoltsCurrents<T>::VoltsCurrents object
 * 
 * @tparam T The base variable minimum-size data type.
 * @param bitsize The variable bitsize within the SM72445 register.
 * @param reg Data formatted as stored in the SM72445 register.
 */
template<typename T>
SM72445::VoltsCurrents<T>::VoltsCurrents(const uint8_t bitsize, uint64_t reg) :
	SM72445::QuadDataStruct<T>::QuadDataStruct(bitsize, reg) {}

/**
 * @brief Override of assignment operator to facilitate the use of references within the structure.
 * 
 * @tparam T The base variable minimum-size data type.
 * @param vc The structure being assigned from.
 * @return SM72445::VoltsCurrents<T>& Reference to the result structure.
 * 
 * @note VoltsCurrents introduces no new members except the references, and therefore this operator simply
 * 		 delegates to the base class assignment operator. The references shall never change from their default.
 */
template <typename T>
SM72445::VoltsCurrents<T> & SM72445::VoltsCurrents<T>::operator = (const VoltsCurrents<T> & vc){
	QuadDataStruct<T>::operator=(vc);
	return * this;
}

/**
 * @brief Construct a new SM72445::ADCs::ADCs object
 * 
 * @note Default constructor.
 */
SM72445::ADCs::ADCs() : 
	SM72445::ADCs::ADCs(0x0ul) {}

/**
 * @brief Construct a new SM72445::ADCs::ADCs object
 * 
 * @param reg Data formatted as stored in the SM72445 register.
 */
SM72445::ADCs::ADCs(uint64_t reg) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, reg) {}

/**
 * @brief Override of assignment operator to facilitate the use of references within the structure.
 * 
 * @param adc The structure being assigned from.
 * @return SM72445::ADCs& Reference to the result structure.
 * 
 * @note ADCs introduces no new members except the references, and therefore this operator simply
 * 		 delegates to the base class assignment operator. The references shall never change from their default.
 */
SM72445::ADCs & SM72445::ADCs::operator=(const ADCs & adc){
	QuadDataStruct<uint16_t>::operator=(adc);
	return * this;
}

/**
 * @brief Construct a new SM72445::Sensors::Sensors object
 * 
 * @note Default constructor.
 */
SM72445::Sensors::Sensors() : 
	SM72445::VoltsCurrents<uint16_t>::VoltsCurrents(10u, 0, 0, 0, 0) {}

/**
 * @brief Construct a new SM72445::Sensors::Sensors object
 * 
 * @param reg Data formatted as stored in the SM72445 register.
 */
SM72445::Sensors::Sensors(uint64_t reg) : 
	SM72445::VoltsCurrents<uint16_t>::VoltsCurrents(10u, reg) {}

/**
 * @brief Override of assignment operator to facilitate the use of references within the structure.
 * 
 * @param sensor The structure being assigned from.
 * @return SM72445::Sensors& Reference to the result structure.
 * 
 * @note Sensors introduces no new members except the references, and therefore this operator simply
 * 		 delegates to the base class assignment operator. The references shall never change from their default.
 */
SM72445::Sensors & SM72445::Sensors::operator = (const Sensors & sensor){
	VoltsCurrents<uint16_t>::operator=(sensor);
	return * this;
}

/**
 * @brief Construct a new SM72445::Offsets::Offsets object
 * 
 * @param vOut
 * @param iOut
 * @param vIn
 * @param iIn
 */
SM72445::Offsets::Offsets(int8_t vOut, int8_t iOut, int8_t vIn, int8_t iIn) :
	SM72445::VoltsCurrents<int8_t>::VoltsCurrents(8u, vOut, iOut, vIn, iIn) {}

/**
 * @brief Construct a new SM72445::Offsets::Offsets object
 * 
 * @note Default constructor.
 */
SM72445::Offsets::Offsets() : 
	SM72445::VoltsCurrents<int8_t>::VoltsCurrents(8u, 0, 0, 0, 0) {}

/**
 * @brief Construct a new SM72445::Offsets::Offsets object
 * 
 * @param reg Data formatted as stored in the SM72445 register.
 */
SM72445::Offsets::Offsets(uint64_t reg) : 
	SM72445::VoltsCurrents<int8_t>::VoltsCurrents(8u, reg) {}

/**
 * @brief Override of assignment operator to facilitate the use of references within the structure.
 * 
 * @param offsets The structure being assigned from.
 * @return SM72445::Offsets& Reference to the result structure.
 * 
 * @note Offsets introduces no new members except the references, and therefore this operator simply
 * 		 delegates to the base class assignment operator. The references shall never change from their default.
 */
SM72445::Offsets & SM72445::Offsets::operator=(const Offsets & offsets){
	VoltsCurrents<int8_t>::operator=(offsets);
	return * this;
}

/**
 * @brief Construct a new SM72445::Thresholds::Thresholds object. 
 * 
 * @note Default constructor.
 */
SM72445::Thresholds::Thresholds() :
	SM72445::Thresholds::Thresholds(0, 0, 0, 0) {}

/**
 * @brief Construct a new SM72445::Thresholds::Thresholds object
 * 
 * @param iInHigh
 * @param iInLow
 * @param iOutHigh
 * @param iOutLow
 */
SM72445::Thresholds::Thresholds(uint16_t iInHigh, uint16_t iInLow, uint16_t iOutHigh, uint16_t iOutLow) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, iInHigh, iInLow, iOutHigh, iOutLow) {}

/**
 * @brief Construct a new SM72445::Thresholds::Thresholds object
 * 
 * @param reg Data formatted as stored in the SM72445 register.
 */
SM72445::Thresholds::Thresholds(uint64_t reg) :
	SM72445::QuadDataStruct<uint16_t>::QuadDataStruct(10u, reg) {}

SM72445::Thresholds & SM72445::Thresholds::operator = (const Thresholds & th){ 
	QuadDataStruct<uint16_t>::operator=(th);
	return * this;
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
 * For convenience, the below methods are methods are suitable implementations of the I2C communication methods for
 * STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL) for systems with a single
 * device or multiple SM72445 devices on a common I2C bus.
 *************************************************************************************************************************/

#define SM72445_USE_STM32_HAL_METHODS
#ifdef SM72445_USE_STM32_HAL_METHODS

#include "stm32g0xx_hal.h"	// STM32 HAL driver to be included. Adjust depending on the specific STM32 platform.

extern I2C_HandleTypeDef * sm72445_hi2c;	// Pointer to the HAL I2C handle for the I2C interface that the SM72445 is on.
											// Define this variable in main.cpp.

SM72445::Status SM72445::I2C_MemRead (uint8_t i2cAddr, uint8_t regAddr, uint64_t & rData){
	uint8_t bytes[SM72445_REG_SIZE+1];	// Transmission length byte at byte[0] (unused). See Fig.15 of datasheet.
	HAL_StatusTypeDef halResult =  HAL_I2C_Mem_Read(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE + 1, 500);

	auto smResult = (halResult == HAL_OK) ? Status::OK : Status::FAIL_I2C;

	rData = 0u;
	if(smResult == Status::OK) for(int i = SM72445_REG_SIZE + 1; i > 0; i--) rData = (rData<<8) + bytes[i];

	return smResult;
}

SM72445::Status SM72445::I2C_MemWrite(uint8_t i2cAddr, const uint8_t regAddr, uint64_t data){
	uint8_t bytes[SM72445_REG_SIZE + 1];
	const uint8_t mask = 0xFFu;

	bytes[0] = SM72445_REG_SIZE;														// Transmission length byte
	for(int i = 1; i < SM72445_REG_SIZE + 1; i++) bytes[i] = (data >> 8*(i-1)) & mask;	// Packing Register Data

	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(sm72445_hi2c, (i2cAddr<<1), regAddr, 1, bytes, SM72445_REG_SIZE + 1, 500);
	return (halResult == HAL_OK) ? SM72445::Status::OK : SM72445::Status::FAIL_I2C;
}

#endif /* SM72445_USE_STM32_HAL_METHODS */

/* End Source Code */


/*** END OF FILE ***/

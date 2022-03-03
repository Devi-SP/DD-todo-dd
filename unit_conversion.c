/******************************************************************************
 *  Copyright ARMSTRONG FLUID TECHNOLOGY
 *  All rights reserved. Reproduction or transmission of this file, or a portion
 *  thereof, is forbidden prior written permission of ARMSTRONG FLUID TECHNOLOGY
 *
 *  File Name   : unit_conversion.c
 *  AUTHOR(S)   : SHARAN N
 *  Function    : Contains different unit conversions
 *  Created     : 2020
 *  VERSION      : 0.3.0
 *  Description :
 *
 *****************************************************************************/
/******************************************************************************
 * Global Includes
 *****************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "unit_conversion.h"
//#include "Application/app_global.h"
//#include "Application/app_base_code/app_global_base_code.h"
//#ifdef LCDScreens
//#include "Screen/app_screen_stringid.h"
//#endif

/******************************************************************************
 * Define / Constants
 *****************************************************************************/
#define CONVERT_KPA_FT_CONST    0.3345623
#define CONVERT_BAR_FT_CONST    0.0298898
#define CONVERT_PSI_FT_CONST    0.4335149
#define CONVERT_M_FT_CONST      0.3047995
#define CONVERT_LS_GPM_CONST    0.0630902
#define CONVERT_M3H_GPM_CONST   0.2271247
#define CONVERT_HP_KWATTS_CONST 0.7456999

/******************************************************************************
 * FUNCTION NAME:  ConvertFreqtoRPM
 *
 * DESCRIPTION:
 * Convert motor drive frequency to a RPM value.
 *
 * This function should only be used for IVS102.  Currently by default the
 * number of poles for IVS102 is 4 until it is read from the VSD.
 *
 * PARAMETERS:
 * frequency    float   the value is in Hz
 *
 * RETURN VALUE:
 * uint16_t     converted value (RPM)
 *
 * NOTES:
 * This function should not be called if a Lafert VFD is used.
 *
 *****************************************************************************/
static uint16_t ConvertFreqtoRPM(float frequency) {
	if (config.field.vfd.manufacturer == VFD_LAFERT) {
		return 0;
	} else {
		return (uint16_t) ((120.0 * frequency) / config.field.vfd.num_poles);
	}
}

/******************************************************************************
 * FUNCTION NAME:  ConvertRPMtoFreq
 *
 * DESCRIPTION:
 * Convert motor drive frequency to a RPM value. This function should only be
 * for used Danfoss. currently by default the no. of poles for Danfoss is 4
 * and is not editable by the user in any way. Noumber of poles are read from
 * VFD drive.
 *
 * PARAMETERS:
 * rpm      uint16_t    value in RPM
 *
 * RETURN VALUE:
 * float    converted value (Hz)
 *
 * NOTES:  Should not use this when the VFD manufacturer is Lafert because
 * it does not have any poles.
 *
 *****************************************************************************/
static float ConvertRPMtoFreq(uint16_t rpm) {
	if (config.field.vfd.manufacturer == VFD_LAFERT) {
		return 0.0;
	} else {
		return ((float) (rpm * config.field.vfd.num_poles) / 120.0);
	}
}

/******************************************************************************
 * FUNCTION NAME:  ConvertRPMtoPCT
 *
 * DESCRIPTION:
 * Convert motor RPM to a percentage value based on the max speed parameter.
 *
 * PARAMETERS:
 * rpm          uint16_t    value in rpm
 * max_speed    uint16_t    high speed limit of the pump
 *
 * RETURN VALUE:
 * float     speed in percentage
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertRPMtoPCT(uint16_t rpm, uint16_t max_speed) {
	return ((float) (rpm * 100) / max_speed);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPCTtoRPM
 *
 * DESCRIPTION:
 * Convert motor running percent (PCT) to RPM based on the high speed.
 *
 * PARAMETERS:
 * percent      float       % value to be converted to RPM
 * max_speed    uint16_t    max user speed of the pump
 *
 * RETURN VALUE:
 * uint16_t      converted value (RPM)
 *
 * NOTES:
 *
 *****************************************************************************/
static uint16_t ConvertPCTtoRPM(float pct, uint16_t max_speed) {
	return (uint16_t) (pct * max_speed / 100);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertKWattstoHorsepower
 *
 * DESCRIPTION:
 * Convert motor power (watts) to motor horsepower. There are ~746 watts in 1hp
 * (hydraulic horsepower).
 *
 * PARAMETERS:
 * watts    float   Value in kilo watts
 *
 * RETURN VALUE:
 * float    Value in HP
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertKWattstoHorsepower(float kwatts) {
	return (kwatts / CONVERT_HP_KWATTS_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertHorsepowerToKWatts
 *
 * DESCRIPTION:
 * Convert motor horsepower to power (kwatts). There are ~746 watts in
 * 1 hp (hydraulic horsepower).
 *
 * PARAMETERS:
 * horsepower   float   value in HP
 *
 * RETURN VALUE:
 * float    value in Kwatts
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertHorsepowerToKWatts(float horsepower) {
	return (horsepower * CONVERT_HP_KWATTS_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertFahrenheitToCelsius
 *
 * DESCRIPTION:
 * Convert Fahrenheit to Celsius.
 *
 * PARAMETERS:
 * fahrenheit   float   value in F degrees
 *
 * RETURN VALUE:
 * float    temperature in degree C
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertFahrenheitToCelsius(float fahrenheit) {
	return ((fahrenheit - 32.0) * (5.0 / 9.0));
}

/******************************************************************************
 * FUNCTION NAME:  ConvertCelsiusToFahrenheit
 *
 * DESCRIPTION:
 * Convert Celsius to Fahrenheit.
 *
 * PARAMETERS:
 * celsius      float   value in degree celsius
 *
 * RETURN VALUE:
 * float        temperature in fahrenheit
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertCelsiusToFahrenheit(float celsius) {
	return ((celsius * (9.0 / 5.0)) + 32.0);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureKPAToFT
 *
 * DESCRIPTION:
 * Convert KPA to foot of water column.
 *
 * PARAMETERS:
 * kpa  float   value in kpa
 *
 * RETURN VALUE:
 * float    value in FT
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureKPAToFT(float kpa) {
	return (kpa * CONVERT_KPA_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureFTToKPA
 *
 * DESCRIPTION:
 * Convert foot of water column to Kpa.
 *
 * PARAMETERS:
 * ft   float   value in FT
 *
 * RETURN VALUE:
 * float    value in Kpa
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureFTToKPA(float ft) {
	return (ft / CONVERT_KPA_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureBARToFT
 *
 * DESCRIPTION:
 * Convert BAR to FT.
 *
 * PARAMETERS:
 * BAR  float   value in BAR
 *
 * RETURN VALUE:
 * float    value in FT
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureBARToFT(float bar) {
	return (bar / CONVERT_BAR_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureFTToBAR
 *
 * DESCRIPTION:
 * Convert FT to BAR.
 *
 * PARAMETERS:
 * ft   float   value in FT
 *
 * RETURN VALUE:
 * float    value in BAR
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureFTToBAR(float ft) {
	return (ft * CONVERT_BAR_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressurePSIToFT
 *
 * DESCRIPTION:
 * Convert PSI to FT.
 *
 * PARAMETERS:
 * psi  float   value in PSI
 *
 * RETURN VALUE:
 * float value in FT
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressurePSIToFT(float psi) {
	return (psi / CONVERT_PSI_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureFTToPSI
 *
 * DESCRIPTION:
 * Convert FT to PSI.
 *
 * PARAMETERS:
 * ft   float   value in FT
 *
 * RETURN VALUE:
 * float    value in PSI
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureFTToPSI(float ft) {
	return (ft * CONVERT_PSI_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureMToFT
 *
 * DESCRIPTION:
 * Convert M to FT.
 *
 * PARAMETERS:
 * m    float   value in m
 *
 * RETURN VALUE:
 * float    value in FT
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureMToFT(float m) {
	return (m / CONVERT_M_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertPressureFTToM
 *
 * DESCRIPTION:
 * Convert FT to M.
 *
 * PARAMETERS:
 * ft   float   value in ft
 *
 * RETURN VALUE:
 * float    value in M
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertPressureFTToM(float ft) {
	return (ft * CONVERT_M_FT_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertFlowLSToGPM
 *
 * DESCRIPTION:
 * Convert flow from LS to GPM.
 *
 * PARAMETERS:
 * ls   float   value in LS
 *
 * RETURN VALUE:
 * float    value in GPM
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertFlowLSToGPM(float ls) {
	return (ls / CONVERT_LS_GPM_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertFlowGPMToLS
 *
 * DESCRIPTION:
 * Convert GPM to LS.
 *
 * PARAMETERS:
 * gpm  float   value in gpm
 *
 * RETURN VALUE:
 * float    value in LS
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertFlowGPMToLS(float gpm) {
	return (gpm * CONVERT_LS_GPM_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertFlowM3HToGPM
 *
 * DESCRIPTION:
 * Convert flow from M^3H to GPM.
 *
 * PARAMETERS:
 * m3h  float   value in m3h
 *
 * RETURN VALUE:
 * float    value in GPM
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertFlowM3HToGPM(float m3h) {
	return (m3h / CONVERT_M3H_GPM_CONST);
}

/******************************************************************************
 * FUNCTION NAME:  ConvertFlowGPMToM3H
 *
 * DESCRIPTION:
 * Convert flow from GPM to M^3H.
 *
 * PARAMETERS:
 * gpm  float   value in gpm
 *
 * RETURN VALUE:
 * float    value in M^3H
 *
 * NOTES:
 *
 *****************************************************************************/
static float ConvertFlowGPMToM3H(float gpm) {
	return (gpm * CONVERT_M3H_GPM_CONST);
}
/******************************************************************************
 * FUNCTION NAME: ConverToInt16
 *
 * DESCRIPTION: Convert the measured value from float (32 bits) to int 16 format
 * assuming the measured value is positive and min value is 0;
 *
 * PARAMETERS:
 * min                  float   min ref. value
 * max                  float   max ref. value
 * measuredValue        float   measured value
 * manufacturer         float   manufactuerer constant
 * conversion_factor    float   conversion factor
 *
 * RETURN VALUE:
 * uint16_T  format of the input value
 *
 * NOTES:
 *
 *****************************************************************************/
uint16_t ConvertFloatToUint16(
        float min,
        float max,
        float measured_value,
        float manufacturer,
        float conversion_factor) {
	return (uint16_t) (measured_value * manufacturer / (max - min)
	        * (float) pow(10, conversion_factor));
}

/******************************************************************************
 * FUNCTION NAME: ConvertRampTimeToRampParameter
 *
 * DESCRIPTION:
 * Convert the ramp time in seconds to ramp parameter used with internal units.
 * This function is only used for a iECM drive.
 *
 * PARAMETERS:
 * ramp_time    float   value of ramp in time
 *
 * RETURN VALUE:
 * uint16_t     ramp parameter
 *
 * NOTES:
 *
 *****************************************************************************/
uint16_t ConvertRampTimeToRampParameter(float ramp_time) {
	float result = 0.0;
	
	if (config.field.vfd.manufacturer == VFD_LAFERT) {
		float rated_speed = (float) config.field.vfd.motor_high_speed / 1.1;
		result = ((10000 * 32768 * ramp_time)
		        / (4 * LAFERT_NOMINAL_SPEED_FACTOR * rated_speed));
	}
#ifdef TEST_DATA
    testdata.field.ramp_calc.rated_speed = rated_speed;
    testdata.field.ramp_calc.ramp_param_calc = result;
#endif
	return (uint16_t) result;
}

/******************************************************************************
 * FUNCTION NAME: ConvertPressure
 *
 * DESCRIPTION:
 * Convert the current unit value (user unit) to FT (default storing unit) or
 * convert FT to the user unit.
 *
 * PARAMETERS:
 * value                float               Value to be converted.
 * convert_value        conv_pressure_e     unit to convert
 *
 * RETURN VALUE:
 * float    converted value
 *
 * NOTES:
 * In the future, this function can be changed to be more explicit on the
 * converstion by having three parameters: value, convert_from and convert_to.
 * This will provide a general function that allow conversions from any type
 * to any other type.
 *
 * If the unit conversion is not handled in the function, an error message is
 * printed out to the console for user notification.
 *****************************************************************************/
//float LoadConvertPressure(float value, conv_pressure_e convert_value) {
//
//	float converted_value = 0;
//
//	switch (convert_value) {
//	case CURRENT_UNIT_TO_FT:
//		switch (config.pressure_unit[1]) {
//		case PRESSURE_BAR:
//			converted_value = ConvertPressureBARToFT(value);
//			break;
//		case PRESSURE_KPA:
//			converted_value = ConvertPressureKPAToFT(value);
//			break;
//		case PRESSURE_PSI:
//			converted_value = ConvertPressurePSIToFT(value);
//			break;
//		case PRESSURE_M:
//			converted_value = ConvertPressureMToFT(value);
//			break;
//		case PRESSURE_FT:
//			converted_value = value;
//			break;
//		default:
//			LOG("Unit conversion not supported.");
//			break;
//		}
//		break;
//	case FT_TO_CURRENT_UNIT:
//		switch (dehxconfig.config.pressure_unit[1]) {
//		case PRESSURE_BAR:
//			converted_value = ConvertPressureFTToBAR(value);
//			break;
//		case PRESSURE_KPA:
//			converted_value = ConvertPressureFTToKPA(value);
//			break;
//		case PRESSURE_PSI:
//			converted_value = ConvertPressureFTToPSI(value);
//			break;
//		case PRESSURE_M:
//			converted_value = ConvertPressureFTToM(value);
//			break;
//		case PRESSURE_FT:
//			converted_value = value;
//			break;
//		default:
//			LOG("Unit conversion not supported.");
//			break;
//		}
//		break;
//	case BAR_TO_FT:
//		converted_value = ConvertPressureBARToFT(value);
//		break;
//	case KPA_TO_FT:
//		converted_value = ConvertPressureKPAToFT(value);
//		break;
//	case PSI_TO_FT:
//		converted_value = ConvertPressurePSIToFT(value);
//		break;
//	case M_TO_FT:
//		converted_value = ConvertPressureMToFT(value);
//		break;
//	default:
//		LOG("Unit conversion not supported.");
//		break;
//	}
//	return converted_value;
//}

/******************************************************************************
 * FUNCTION NAME: ConvertPressureString
 *
 * DESCRIPTION:
 * Gets the current pressure unit and returns the pressure unit stringID
 *
 * PARAMETERS:
 *
 * RETURN VALUE:
 * DictionaryString_e   pressure unit strinID
 *
 * NOTES:
 *
 *****************************************************************************/
#ifdef LCDScreens
DictionaryString_e ConvertPressureString() {
	DictionaryString_e pressure_unit_string;
	
	switch (BC_config.field.units.pressure) {
	case PRESSURE_BAR:
		pressure_unit_string = STRING_BAR;
		break;
	case PRESSURE_KPA:
		pressure_unit_string = STRING_KPA;
		break;
	case PRESSURE_PSI:
		pressure_unit_string = STRING_PSI;
		break;
	case PRESSURE_M:
		pressure_unit_string = STRING_M;
		break;
	case PRESSURE_FT:
	default:
		pressure_unit_string = STRING_FT;
		break;
	}
	return pressure_unit_string;
}
#endif
/******************************************************************************
 * FUNCTION NAME: ConvertFlow
 *
 * DESCRIPTION:
 * Convert the current unit value (user unit) to GPM (default storing unit) or
 * convert GPM to the user unit.
 *
 * PARAMETERS:
 * value                float           Value to be converted.
 * convert_value        conv_flow_e     unit to convert
 *
 * RETURN VALUE:
 * float    converted value
 *
 * NOTES:
 * In the future, this function can be changed to be more explicit on the
 * converstion by having three parameters: value, convert_from and convert_to.
 * This will provide a general function that allow conversions from any type
 * to any other type.
 *
 * If the unit conversion is not handled in the function, an error message is
 * printed out to the console for user notification.
 *****************************************************************************/
//float ConvertFlow(float value, conv_flow_e convert_value) {
//
//	float converted_value = 0;
//
//	switch (convert_value) {
//	case CURRENT_UNIT_TO_GPM:
//		switch (config.field.units.flow) {
//		case FLOW_LS:
//			converted_value = ConvertFlowLSToGPM(value);
//			break;
//		case FLOW_M3H:
//			converted_value = ConvertFlowM3HToGPM(value);
//			break;
//		case FLOW_GPM:
//			converted_value = value;
//			break;
//		}
//		break;
//	case GPM_TO_CURRENT_UNIT:
//		switch (BC_config.field.units.flow) {
//		case FLOW_LS:
//			converted_value = ConvertFlowGPMToLS(value);
//			break;
//		case FLOW_M3H:
//			converted_value = ConvertFlowGPMToM3H(value);
//			break;
//		case FLOW_GPM:
//			converted_value = value;
//			break;
//		}
//		break;
//	case LS_TO_GPM:
//		converted_value = ConvertFlowLSToGPM(value);
//		break;
//	case M3H_TO_GPM:
//		converted_value = ConvertFlowM3HToGPM(value);
//		break;
//	default:
//		LOG("Unit conversion not supported.");
//		converted_value = 0;
//		break;
//	}
//	return converted_value;
//}

/******************************************************************************
 * FUNCTION NAME: ConvertFlowString
 *
 * DESCRIPTION:
 * Gets the current flow unit and returns the flow unit stringID
 *
 * PARAMETERS:
 *
 * RETURN VALUE:
 * DictionaryString_e   Flow unit stringID
 *
 * NOTES:
 *
 *****************************************************************************/
#ifdef LCDScreens
DictionaryString_e ConvertFlowString() {
	DictionaryString_e flow_unit_string;
	switch (BC_config.field.units.flow) {
	case FLOW_LS:
		flow_unit_string = STRING_LPS;
		break;
	case FLOW_M3H:
		flow_unit_string = STRING_M3H;
		break;
	case FLOW_GPM:
	default:
		flow_unit_string = STRING_GPM;
		break;
	}
	return flow_unit_string;
}
#endif
/******************************************************************************
 * FUNCTION NAME: ConvertPower
 *
 * DESCRIPTION:
 * Convert the power value in KW to current unit or vice-versa.
 *
 * PARAMETERS:
 * value                float           Value to be converted (HP or KW)
 * convert_value        conv_power_e    unit to convert
 *
 * RETURN VALUE:
 * float    converted value
 *
 * NOTES:
 * In the future, this function can be changed to be more explicit on the
 * converstion by having three parameters: value, convert_from and convert_to.
 * This will provide a general function that allow conversions from any type
 * to any other type.
 *
 * If the unit conversion is not handled in the function, an error message is
 * printed out to the console for user notification.
 *****************************************************************************/
//float ConvertPower(float value, conv_power_e convert_value) {
//	float converted_value = 0;
//
//	switch (convert_value) {
//	case CURRENT_UNIT_TO_KW:
//		switch (BC_config.field.units.power) {
//		case POWER_HP:
//			converted_value = ConvertHorsepowerToKWatts(value);
//			break;
//		case POWER_KW:
//		default:
//			converted_value = value;
//			break;
//		}
//		break;
//	case KW_TO_CURRENT_UNIT:
//		switch (BC_config.field.units.power) {
//		case POWER_HP:
//			converted_value = ConvertKWattstoHorsepower(value);
//			break;
//		case POWER_KW:
//		default:
//			converted_value = value;
//			break;
//		}
//		break;
//	case HP_TO_KW:
//		converted_value = ConvertHorsepowerToKWatts(value);
//		break;
//	default:
//		LOG("Unit conversion not supported.");
//		converted_value = 0;
//		break;
//	}
//
//	return converted_value;
//}

/******************************************************************************
 * FUNCTION NAME: ConvertPowerString
 *
 * DESCRIPTION:
 * Gets the current power unit and returns the power unit stringID
 *
 * PARAMETERS:
 *
 * RETURN VALUE:
 * DictionaryString_e   power unit stringID
 *
 * NOTES:
 *
 *****************************************************************************/
#ifdef LCDScreens
DictionaryString_e ConvertPowerString() {
	DictionaryString_e power_unit_string;
	
	switch (BC_config.field.units.power) {
	case POWER_HP:
		power_unit_string = STRING_HP;
		break;
	case POWER_KW:
	default:
		power_unit_string = STRING_KW;
		break;
	}
	return power_unit_string;
}
#endif
/******************************************************************************
 * FUNCTION NAME: ConvertSpeed
 *
 * DESCRIPTION:
 * Convert the speed value in RPM to current unit
 *
 * PARAMETERS:
 * value                float           Value to be converted
 * convert_value        conv_speed_e    unit to convert
 *
 * RETURN VALUE:
 * float    converted value
 *
 * NOTES:
 * In the future, this function can be changed to be more explicit on the
 * converstion by having three parameters: value, convert_from and convert_to.
 * This will provide a general function that allow conversions from any type
 * to any other type.
 *
 * If the unit conversion is not handled in the function, an error message is
 * printed out to the console for user notification.
 *****************************************************************************/
float ConvertSpeed(float value, conv_speed_e convert_value) {
	float converted_value = 0;
	
	switch (convert_value) {
	case CURRENT_UNIT_TO_RPM:
		switch (BC_config.field.units.speed) {
		case PCT:
			converted_value = ConvertPCTtoRPM(
			    value,
			    config.field.vfd.high_speed_limit);
			break;
		case RPM:
		default:
			converted_value = value;
			break;
		}
		break;
	case RPM_TO_CURRENT_UNIT:
		switch (BC_config.field.units.speed) {
		case PCT:
			converted_value = ConvertRPMtoPCT(
			    value,
			    config.field.vfd.high_speed_limit);
			break;
		case RPM:
		default:
			converted_value = value;
			break;
		}
		break;
	case RPM_TO_PCT:
		converted_value = ConvertRPMtoPCT(
		    value,
		    config.field.vfd.high_speed_limit);
		break;
	case PCT_TO_RPM:
		converted_value = ConvertPCTtoRPM(
		    value,
		    config.field.vfd.high_speed_limit);
		break;
	case FREQ_TO_RPM:
		converted_value = ConvertFreqtoRPM(value);
		break;
	case RPM_TO_FREQ:
		converted_value = ConvertRPMtoFreq(value);
		break;
	default:
		printf("Unit conversion not supported.");
		break;
		
	}
	return converted_value;
}

/******************************************************************************
 * FUNCTION NAME: ConvertSpeedString
 *
 * DESCRIPTION:
 * Gets the current power speed and returns the speed unit stringID
 *
 * PARAMETERS:
 *
 * RETURN VALUE:
 * DictionaryString_e   speed unit stringID
 *
 * NOTES:
 *
 *****************************************************************************/
#ifdef LCDScreens
DictionaryString_e ConvertSpeedString() {
	DictionaryString_e speed_unit_string;
	
	switch (BC_config.field.units.speed) {
	case PCT:
		speed_unit_string = STRING_PCT;
		break;
	case RPM:
	default:
		speed_unit_string = STRING_RPM;
		break;
	}
	return speed_unit_string;
}
#endif
/******************************************************************************
 * FUNCTION NAME: ConvertTemperature
 *
 * DESCRIPTION:
 * Convert the temperature value in Celsius to current unit
 *
 * PARAMETERS:
 * value                float               Value to be converted.
 * convert_value        conv_temperature_e  unit to convert
 *
 * RETURN VALUE:
 * float    converted value.
 *
 * NOTES:
 * In the future, this function can be changed to be more explicit on the
 * converstion by having three parameters: value, convert_from and convert_to.
 * This will provide a general function that allow conversions from any type
 * to any other type.
 *
 * If the unit conversion is not handled in the function, an error message is
 * printed out to the console for user notification.
 *****************************************************************************/
float ConvertTemperature(float value, conv_temperature_e convert_value) {
    float converted_value = 0;

    switch (convert_value) {
    case CURRENT_UNIT_TO_CEL:
        switch (BC_config.field.units.temperature) {
        case FAHRENHEIT:
            converted_value = ConvertFahrenheitToCelsius(value);
            break;
        case CELSIUS:
        default:
            converted_value = value;
            break;
        }
        break;
    case CEL_TO_CURRENT_UNIT:
        switch (BC_config.field.units.temperature) {
        case FAHRENHEIT:
            converted_value = ConvertCelsiusToFahrenheit(value);
            break;
        case CELSIUS:
        default:
            converted_value = value;
            break;
        }
        break;
    default:
        printf("Unit conversion not supported.");
        break;
    }
    return converted_value;
}
//Source Temperature Unit
//float LoadConvertTemperature(float value, conv_temperature_e convert_value) {
//    float converted_value = 0;
//
//    switch (convert_value) {
//    case CURRENT_UNIT_TO_CEL:
//        switch (dehxconfig.config.temperature_unit[1]) {
//        case FAHRENHEIT:
//            converted_value = ConvertFahrenheitToCelsius(value);
//            break;
//        case CELSIUS:
//        default:
//            converted_value = value;
//            break;
//        }
//        break;
//    case CEL_TO_CURRENT_UNIT:
//        switch (dehxconfig.config.temperature_unit[1]) {
//        case FAHRENHEIT:
//            converted_value = ConvertCelsiusToFahrenheit(value);
//            break;
//        case CELSIUS:
//        default:
//            converted_value = value;
//            break;
//        }
//        break;
//    default:
//        LOG("Unit conversion not supported.");
//        break;
//    }
//    return converted_value;
//}

// IEEE Units
int ieee[32] = { 0 };
int32_t ieee754 = 0;
typedef union {
	float f;
	struct {
		unsigned int mantissa :23;
		unsigned int exponent :8;
		unsigned int sign :1;
	} raw;
} myfloat;

myfloat var;
/******************************************************************************
 * FUNCTION NAME:  convertToInt
 *
 * DESCRIPTION:
 * Convert given number in to integer
 *
 * PARAMETERS:
 * integer arr
 * low - quantity
 * high - position

 * RETURN VALUE:
 * INT
 *
 *
 *****************************************************************************/
unsigned int convertToInt(int *arr, int low, int high) {
	unsigned f = 0, i;
	for (i = high; i >= low; i--) {
		f = f + arr[i] * pow(2, high - i);
	}
	return f;
}
/******************************************************************************
 * FUNCTION NAME:  ConvertHexToBin
 *
 * DESCRIPTION:
 * Convert given hex number in to 32 bit binary array
 *
 * PARAMETERS:
 * hex - Hex value
 *
 *
 * RETURN VALUE:
 * NONE
 *
 *
 *****************************************************************************/
void ConvertHexToBin(int32_t hex) {
	
	unsigned int i = 0;
	do {
		ieee[31 - i] = (hex >> i) & 0x01;
		i++;
	} while (i < 32);
	
}
/******************************************************************************
 * FUNCTION NAME:  ConvertIEEE754to32bitFloatValue
 *
 * DESCRIPTION:
 * Convert given IEEE754 floating point representation in to float value
 *
 * PARAMETERS:
 * value - Hex value
 *
 *
 * RETURN VALUE:
 * Float
 *
 *
 *****************************************************************************/
float ConvertIEEE754to32bitFloatValue(int32_t value) {
	//ieee[32] = {0};
	ConvertHexToBin(value);
	unsigned f = convertToInt(ieee, 9, 31);
	var.raw.mantissa = f;
	f = convertToInt(ieee, 1, 8);
	var.raw.exponent = f;
	var.raw.sign = ieee[0];
	return var.f;
}
/******************************************************************************
 * FUNCTION NAME:  ConvertFloatValuetoIEEE754
 *
 * DESCRIPTION:
 * Convert given float value to IEEE754 floating point representation.
 *
 * PARAMETERS:
 * value - float value
 *
 *
 * RETURN VALUE:
 * 32 bit integer
 *
 *
 *****************************************************************************/
int32_t ConvertFloatValuetoIEEE754(float value) {
	var.f = value;
	int k, j;
	ieee754 = 0;
	//  ieee[32] = {0};
	ieee[31] = var.raw.sign;
	for (k = 8 - 1, j = 30; k >= 0; k--, j--)
		ieee[j] = ((var.raw.exponent >> k) & 0x01);
	for (j = 22, k = 23 - 1; k >= 0; k--, j--)
		ieee[j] = ((var.raw.mantissa >> k) & 0x01);
	for (j = 0; j < 32; j++)
		ieee754 = ieee754 + (ieee[j] * pow(2, j));
	return ieee754;
}

//Source Pressure Unit
//float SourceConvertPressure(float value, conv_pressure_e convert_value) {
//
//    float converted_value = 0;
//
//    switch (convert_value) {
//    case CURRENT_UNIT_TO_FT:
//        switch (dehxconfig.config.pressure_unit[0]) {
//        case PRESSURE_BAR:
//            converted_value = ConvertPressureBARToFT(value);
//            break;
//        case PRESSURE_KPA:
//            converted_value = ConvertPressureKPAToFT(value);
//            break;
//        case PRESSURE_PSI:
//            converted_value = ConvertPressurePSIToFT(value);
//            break;
//        case PRESSURE_M:
//            converted_value = ConvertPressureMToFT(value);
//            break;
//        case PRESSURE_FT:
//            converted_value = value;
//            break;
//        default:
//            LOG("Unit conversion not supported.");
//            break;
//        }
//        break;
//    case FT_TO_CURRENT_UNIT:
//        switch (dehxconfig.config.pressure_unit[0]) {
//        case PRESSURE_BAR:
//            converted_value = ConvertPressureFTToBAR(value);
//            break;
//        case PRESSURE_KPA:
//            converted_value = ConvertPressureFTToKPA(value);
//            break;
//        case PRESSURE_PSI:
//            converted_value = ConvertPressureFTToPSI(value);
//            break;
//        case PRESSURE_M:
//            converted_value = ConvertPressureFTToM(value);
//            break;
//        case PRESSURE_FT:
//            converted_value = value;
//            break;
//        default:
//            LOG("Unit conversion not supported.");
//            break;
//        }
//        break;
//    case BAR_TO_FT:
//        converted_value = ConvertPressureBARToFT(value);
//        break;
//    case KPA_TO_FT:
//        converted_value = ConvertPressureKPAToFT(value);
//        break;
//    case PSI_TO_FT:
//        converted_value = ConvertPressurePSIToFT(value);
//        break;
//    case M_TO_FT:
//        converted_value = ConvertPressureMToFT(value);
//        break;
//    default:
//        LOG("Unit conversion not supported.");
//        break;
//    }
//    return converted_value;
//}

//Source Temperature Unit
//float SourceConvertTemperature(float value, conv_temperature_e convert_value) {
//    float converted_value = 0;
//
//    switch (convert_value) {
//    case CURRENT_UNIT_TO_CEL:
//        switch (dehxconfig.config.temperature_unit[0]) {
//        case FAHRENHEIT:
//            converted_value = ConvertFahrenheitToCelsius(value);
//            break;
//        case CELSIUS:
//        default:
//            converted_value = value;
//            break;
//        }
//        break;
//    case CEL_TO_CURRENT_UNIT:
//        switch (dehxconfig.config.temperature_unit[0]) {
//        case FAHRENHEIT:
//            converted_value = ConvertCelsiusToFahrenheit(value);
//            break;
//        case CELSIUS:
//        default:
//            converted_value = value;
//            break;
//        }
//        break;
//    default:
//        LOG("Unit conversion not supported.");
//        break;
//    }
//    return converted_value;
//}

// Added by Pradeep and this function is called from MaximizeSourceSide_DeltaT_VariableLoadSide function

float Unit_Conversion(convert_parameter Conversion_Parameter,float Value, units InputUnit, units OutputUnit) {
     float converted_value = 0;
    switch(Conversion_Parameter){
        case FLUID_CONDUCTIVITY:
         if  (InputUnit == BTUHRFTF && OutputUnit == WMK)
             converted_value = Value * 1.73;
         else
             converted_value = Value * 1;
         break;
        case FLUID_DENSITY:
         if  (InputUnit == LBFT3 && OutputUnit == KGM3)
             converted_value = Value * 1.6019E1;
         else
             converted_value = Value * 1;
         break;
        case FLUID_SPECIFIC_HEAT:
           if  (InputUnit == BTULBF && OutputUnit == JKGK)
             converted_value = Value * 4.1868E3;
           else
               converted_value = Value * 1;
           break;
        case FLUID_VISICOSITY:
         if  (InputUnit == CP && OutputUnit == NSM2)
             converted_value = Value * 1E-3;
        else
            converted_value = Value * 1;
        break;
    case TEMPERATURE:
        if  (InputUnit == F && OutputUnit == C)
            converted_value = ((Value - 32.0) * (5.0/9.0));
        else if  (InputUnit == C && OutputUnit == F)
            converted_value = (( Value  * (9.0 / 5.0)) + 32.0);
        else
           converted_value = Value * 1;
       break;
    case VFLOW:
        if  (InputUnit == USGPM && OutputUnit == M3S)
            converted_value = Value * 6.309E-5;
        else if  (InputUnit == LS && OutputUnit == M3S)
            converted_value = Value * 1E-3;
        else if  (InputUnit == LMIN && OutputUnit == M3S)
            converted_value = Value * 1.66667E-5;
        else if  (InputUnit == LHR && OutputUnit == M3S)
            converted_value = Value * 2.77778E-7;
        else if  (InputUnit == M3HR && OutputUnit == M3S)
            converted_value = Value * 2.77778E-4;
        else if  (InputUnit == M3S  && OutputUnit ==  USGPM)
            converted_value = Value *1.58503E+4;
        else if  (InputUnit == M3S && OutputUnit ==  LS)
            converted_value = Value * 1E+3;
        else if  (InputUnit == M3S && OutputUnit ==  LMIN)
            converted_value = Value * 6.00000E+4;
       else if  (InputUnit == M3S && OutputUnit ==  LHR)
            converted_value = Value * 3.60000E+6;
       else if  (InputUnit == M3S && OutputUnit ==  M3HR)
            converted_value = Value * 3.60000E+3;
       else
           converted_value = Value * 1;
      break;
    case PRESSURE:
        if  (InputUnit == PSI && OutputUnit == PA)
            converted_value = Value * 6.89476E+3;
        else if(InputUnit == FT && OutputUnit ==  PA)
            converted_value = Value * 2.98907E+3;
        else if(InputUnit == M && OutputUnit ==  PA)
            converted_value = Value * 9.80665E+3;
        else if(InputUnit == BAR && OutputUnit ==  PA)
            converted_value = Value * 1E+5;
        else if(InputUnit == KPA && OutputUnit ==  PA)
            converted_value = Value * 1E+3;
        else if(InputUnit == PA && OutputUnit ==  PSI)
            converted_value = Value * 1.45038E-4;
        else if(InputUnit == PA && OutputUnit ==  FT)
            converted_value = Value * 3.34553E-4;
        else if(InputUnit == PA && OutputUnit ==  M)
            converted_value = Value * 1.01972E-4;
        else if(InputUnit == PA && OutputUnit ==  BAR)
            converted_value = Value * 1E-5;
        else if(InputUnit == PA && OutputUnit ==  KPA)
            converted_value = Value * 1E-3;
        else
            converted_value = Value * 1;
        break;
    case HEAT:
        if(InputUnit == BTUHR && OutputUnit == W)
            converted_value = Value * 2.93071E-1;
        else if(InputUnit == KCALHR && OutputUnit ==W)
            converted_value = Value * 1.16300E+0;
        else if(InputUnit == KW && OutputUnit ==  W)
            converted_value = Value * 1E+3;
        else if(InputUnit == MBH && OutputUnit ==  W)
            converted_value = Value * 2.93071E+2;
        else if(InputUnit == W && OutputUnit ==  BTUHR)
            converted_value = Value * 3.412142;
        else if(InputUnit == W && OutputUnit ==  KCALHR)
            converted_value = Value * 8.59845E-1;
        else if(InputUnit == W && OutputUnit ==  KW)
            converted_value = Value * 1E-3;
        else if(InputUnit == W && OutputUnit ==  MBH)
            converted_value = Value * 3.41214E-3;
        else
            converted_value = Value * 1;
        break;
    case FOULING:
        if(InputUnit == FT2FHRBTU && OutputUnit ==  M2KW)
            converted_value = Value * 1.76110E-1;
        else if(InputUnit == M2KKW && OutputUnit ==  M2KW)
            converted_value = Value * 1E3;
        else if(InputUnit == M2KW && OutputUnit ==  FT2FHRBTU)
            converted_value = Value * 5.67826 ;
        else if(InputUnit == M2KW && OutputUnit ==  M2KKW)
            converted_value = Value * 1E-3;
        else
            converted_value = Value * 1;
        break;
    }
    return converted_value;
}


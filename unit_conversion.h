/******************************************************************************
 *  Copyright ARMSTRONG FLUID TECHNOLOGY
 *  All rights reserved. Reproduction or transmission of this file, or a portion
 *  thereof, is forbidden prior written permission of ARMSTRONG FLUID TECHNOLOGY
 *
 *  File Name   : unit_conversion.h
 *  AUTHOR(S)   : SHARAN N
 *  Function    : header file for unit_conversion.c
 *  Created     : 2020
 *  VERSION      : 0.3.0
 *  Description :
 *
 *****************************************************************************/

#ifndef APPLICATION_IEEE754_H_
#define APPLICATION_IEEE754_H_

//#include "Application/app_base_code/app_global_base_code.h"
//#include "Application/app_global.h"
//#include "Screen/app_screen_stringid.h"
#include <stdbool.h>
static float ConvertRPMtoFreq(uint16_t rpm);
float ConvertIEEE754to32bitFloatValue(int32_t value);
int32_t ConvertFloatValuetoIEEE754(float value);

/******************************************************************************
 * Define / Constants
 *****************************************************************************/
//used in vfdmodbus.c file, since its converting factor
#define LAFERT_MAX_SPEED_FACTOR         32268
//right now not used else where, but can be used in future.
#define LAFERT_NOMINAL_SPEED_FACTOR     29334

/******************************************************************************
 * TypeDefs
 *****************************************************************************/
//The enumeration test is of the form X_TO_Y where X is the unit to go from
//and Y is the unit to convert to.
typedef enum {
	CURRENT_UNIT_TO_FT,
	FT_TO_CURRENT_UNIT,
	BAR_TO_FT,
	KPA_TO_FT,
	PSI_TO_FT,
	M_TO_FT,
} conv_pressure_e;

//The enumeration test is of the form X_TO_Y where X is the unit to go from
//and Y is the unit to convert to.
typedef enum {
	CURRENT_UNIT_TO_GPM,
	GPM_TO_CURRENT_UNIT,
	LS_TO_GPM,
	M3H_TO_GPM,
} conv_flow_e;

//The enumeration test is of the form X_TO_Y where X is the unit to go from
//and Y is the unit to convert to.
typedef enum {
	CURRENT_UNIT_TO_RPM,
	RPM_TO_CURRENT_UNIT,
	RPM_TO_PCT,
	PCT_TO_RPM,
	FREQ_TO_RPM,
	RPM_TO_FREQ,
} conv_speed_e;

//The enumeration test is of the form X_TO_Y where X is the unit to go from
//and Y is the unit to convert to.
typedef enum {
	CURRENT_UNIT_TO_KW,
	KW_TO_CURRENT_UNIT,
	HP_TO_KW,
} conv_power_e;

//The enumeration test is of the form X_TO_Y where X is the unit to go from
//and Y is the unit to convert to.
typedef enum {
	CURRENT_UNIT_TO_CEL,
	CEL_TO_CURRENT_UNIT,
} conv_temperature_e;

//Added by Pradeep
typedef enum{
    FLUID_CONDUCTIVITY,
    FLUID_DENSITY,
    FLUID_SPECIFIC_HEAT,
    FLUID_VISICOSITY,
    TEMPERATURE,
    VFLOW,
    PRESSURE,
    HEAT,
    FOULING,

}convert_parameter ;

typedef enum {

    C,
    F,

    USGPM,
    LS,
    LMIN,
    LHR,
    M3HR,
    M3S,

    PSI,
    FT,
    M,
    BAR,
    KPA,
    PA,

    BTUHR,
    KCALHR,
    KW,
    MBH,
    W,

    FT2FHRBTU,
    M2KKW,
    M2KW,

    BTUHRFTF,
    WMK, //CONDUCTIVITY,
    LBFT3,
    KGM3,  //DENSITY,
    BTULBF,
    JKGK,  // SPECIFIC_HEAT,
    CP,
    NSM2,     //VISICOSITY,
} units;


typedef enum {
    MONTH_DAY_YEAR,
    DAY_MONTH_YEAR, // internal storage type
} date_format_e;

typedef enum {
    HR_12,
    HR_24, // internal storage type
} time_format_e;

typedef enum {
    FAHRENHEIT,
    CELSIUS, // internal storage type
} temperature_format_e;

typedef enum {
    PRESSURE_PSI,
    PRESSURE_FT,
    PRESSURE_M,
    PRESSURE_BAR, // internal storage type
    PRESSURE_KPA,
    PRESSURE_PA,
} pressure_format_e;

typedef enum {
    FLOW_LS, //liters per second
    FLOW_M3H, //cubic meters per hour
    FLOW_GPM, //gallons per minte - internal storage type
} flow_format_e;

typedef enum {
    POWER_HP,
    POWER_KW,
} power_format_e;

typedef enum {
    RPM, // internal storage type
    PCT, //% of maximum speed
} speed_e;

/******************************************************************************
 *  Copyright ARMSTRONG FLUID TECHNOLOGY
 *  All rights reserved. Reproduction or transmission of this file, or a portion
 *  thereof, is forbidden prior written permission of ARMSTRONG FLUID TECHNOLOGY
 *
 *  File Name   : app_global.h
 *  AUTHOR(S)   : MATHAN
 *  Function    :
 *  Created     : 2020
 *  VERSION      : 0.4.0
 *  Description : Global file for application specific variables
 *
 *****************************************************************************/
#ifndef APPLICATION_GLOBAL_H
#define APPLICATION_GLOBAL_H
//#include "Application/app_base_code/app_global_base_code.h"
// Firmware Version
#define FW_VERSION_MAJOR    "01"    // major build number
#define FW_VERSION_MINOR    "00"    //minor build number
#define FW_VERSION_PATCH    "00"    //patch build number

#define IPS_NO_OF_PUMPS         3
/******************************************************************************
 * Defines maximum number of slaves to communicate with master port COM1.
 *****************************************************************************/
#define MAX_NO_OF_PARA_SET      2
#define MAX_NO_OF_SLAVES        0x06
#define MIN_NO_OF_SLAVE         0x01

#define LAFERT_DEVELOPER_LOW_SPEED      1000    //in RPM - changed to 1500 from 1000 for parallel
#define DANFOSS_DEVELOPER_LOW_SPEED     420     //in RPM

#define MAX_SCHEDULE_LIST           10  // the size of the number of schedule items

#define TIMER_SCREEN                (5*60*60*6)    //every 6hours restart the screen one time

// bit field pump tests for variable pump_tests
#define PUMP_CYCLE_TEST         0
#define PUMP_SENSORLESS_TEST    1

#define NUM_LEDS            3

uint8_t modbus_no_of_slaves;
uint8_t modbus_device_com_fail[6];
uint8_t test_task_switch;
uint8_t HomePumpStatusButton;
bool trace;
bool key1;
int TrendIndex;
bool DataDelete;
bool splash_permissive;
bool Ethernet_Mode;
bool STA_netconfig_done;
bool wifi_popup_holdset;

///testing


struct tm
{
    int tm_sec;      /* seconds after the minute   - [0,59]  */
    int tm_min;      /* minutes after the hour     - [0,59]  */
    int tm_hour;     /* hours after the midnight   - [0,23]  */
    int tm_mday;     /* day of the month           - [1,31]  */
    int tm_mon;      /* months since January       - [0,11]  */
    int tm_year;     /* years since 1900                     */
    int tm_wday;     /* days since Sunday          - [0,6]   */
    int tm_yday;     /* days since Jan 1st         - [0,365] */
    int tm_isdst;    /* Daylight Savings Time flag           */
#if defined(_AEABI_PORTABILITY_LEVEL) && _AEABI_PORTABILITY_LEVEL != 0
    int __extra_1, __extra_2;            /* ABI-required extra fields */
#endif

};
typedef unsigned int __time32_t; //copied from TI-cgt-arm - ccs tools
typedef __time32_t time_t;
///testing

// operating state of the pump
typedef enum {
    PUMP_STOP = 0,
    PUMP_START = 1,
}__attribute__ ((packed))
pump_start_e;

// this enum is used to set what the vfd will do when powered up
typedef enum {
    PUMP_STARTUP_RESUME,
    PUMP_STARTUP_STOP,
} pump_startup_e;

// automatic pump control modes
typedef enum {
    PUMP_CONSTANT_FLOW = 1,
    PUMP_CONSTANT_PRESSURE = 2,
    PUMP_LINEAR_PRESSURE = 3,
    PUMP_QUAD_PRESSURE = 4,
    PUMP_QUAD_PRESSURE_MAX_FLOW = 6,
    PUMP_QUAD_PRESSURE_MIN_FLOW = 5,
    PUMP_QUAD_PRES_MIN_MAX_FLOW = 7,
    DUAL_SEASON_SELECT_MODE = 8,
    DUAL_SEASON_SELECT_SUB = 9,
} pump_auto_mode_e;

// Parallel pump control modes
typedef enum {
    PUMP_P_CONSTANT_FLOW = 1,
    PUMP_P_CONSTANT_PRESSURE = 2,
    PUMP_P_LINEAR_PRESSURE = 3,
    PUMP_P_QUAD_PRESSURE = 4,
    PUMP_P_QUAD_PRESSURE_MAX_FLOW = 6,
    PUMP_P_QUAD_PRESSURE_MIN_FLOW = 5,
    PUMP_P_QUAD_PRES_MIN_MAX_FLOW = 7,
    PUMP_P_INPUTS = 8,
//DUAL_SEASON_SELECT_SUB = 9,
} pump_parallel_mode_e;

typedef enum {          //this refers to the speed control
    PUMP_PARALLEL,          //inter-card communication
    PUMP_INPUTS,            //Inputs from VFD
    PUMP_MANUAL,            //local control of pump constant speed
    PUMP_AUTOMATIC,     // local control of pump modes except for constant speed
    PUMP_REMOTE,            //BMS/BAS communication commands
} pump_control_mode_e;

// operating state of the pump
typedef enum {
//    DI_IN_ENABLED_LOW = 0,  //stop pump with digital input 1 (or 2) enabled but HW I/O pin disconnected
//    DI_IN_ENABLED_HIGH = 1, //digital input 1 (or 2): enabled and HW I/O pin connected to high
    DI_IN_NO_FUNCTION = 0,//both digital input 1 and 2 disabled
    DI_IN_ON_OFF_LOW = 1,//stop pump with digital input 1 (or 2) enabled but HW I/O pin disconnected
    DI_IN_ON_OFF_HIGH = 2,//digital input 1 (or 2): enabled and HW I/O pin connected to high
    DI_IN_SET_SPEED_LOW = 3,//HW I/O pin disconnected, pump run in normal auto mode operation
    DI_IN_SET_SPEED_HIGH = 4,//HW I/O pin connected to high, pump run in set speed
}__attribute__ ((packed))
pump_digital_input_state_e;

//TODO: Danfoss 0 = Current, Lafert 0 = Voltage

//#if (config.field.vfd.manufacturer == VFD_DANFOSS)
typedef enum {
    CURRENT = 0,
    VOLTAGE = 1,
} analog_in_config_e;

typedef enum {
    MASTER,
    SLAVE,
} parallel_pump_control_e;

typedef enum {
    ONE_SENSOR_CONT,
    TWO_SENSOR_CONT,
} ai_sensors_control_e;

typedef enum {
    AIPC_NO_FUNCTION,
    AIPC_PRESSURE,
    AIPC_SPEED,
//  AIPC_FLOW,
    AIPC_TEMP_SENSOR,
//  AIPC_SENSOR_HEAD,
} ai_pump_control_e;

typedef enum {
    AOPC_NO_FUNCTION,
    AOPC_SPEED,
    AOPC_FLOW,
    AOPC_BYPASS_VALVE,
} ao_pump_control_e;

typedef enum {
    DIPC_NONE,
    DIPC_ON_OFF,
    DIPC_SET_SPEED,
} di_pump_control_e;

typedef enum {
    DOPC_NO_FUNCTION,
    DOPC_BYPASS_VALVE,
    DOPC_ALARM,
    DOPC_FLOW_THRESHOLD,
    DOPC_HEAD_THRESHOLD,
    DOPC_RUN_STATUS,
} do_pump_control_e;

typedef enum {
    ROPC_NO_FUNCTION,
    ROPC_BYPASS_VALVE,
    ROPC_ALARM,
    ROPC_FLOW_THRESHOLD,
    ROPC_HEAD_THRESHOLD,
    ROPC_RUN_STATUS,
} ro_pump_control_e;

typedef enum {
    OUTPUT_STATE_IDLE,
    OUTPUT_STATE_DIGITAL,
    OUTPUT_STATE_ANALOG,
    OUTPUT_STATE_RELAY,
} output_states;

typedef enum {
    OUTPUTS_DISABLED = 0,
    OUTPUTS_ENABLED = 1,
} output_function_e;

typedef enum {
    MPC_RUN,
    MPC_STOP,
} manual_pump_control_e;

typedef enum {
    ROTATION_CW = 0,
    ROTATION_CCW = 1,
} motor_rotation_e;

typedef enum {
    SKIP_SPEED_RANGE_1,
    SKIP_SPEED_RANGE_2,
    SKIP_SPEED_COUNT,
} skip_speed_ranges_em;

typedef enum {
    HIHG_EFFICIENCY_MODE,     //3KHz
    LOW_NOISE_MODE,           //6KHz
} switching_freq_e;

typedef enum {
    HAND_OFF,
    HAND_HAND,
    HAND_AUTO,
} hand_flag_t;

//Dual Season Setup Active mode
typedef enum {
    STANDARD,     //standard mode
    MODE_1,     //heating mode
    MODE_2,     //cooling mode
} dualsea_mode_e;

// type of vfd used, based on manufacturer
typedef enum {
    VFD_SIMULATOR,
    VFD_LAFERT,
    VFD_DANFOSS,
}__attribute__ ((packed))
vfd_mfg_e;

typedef enum {
    IVS_MSG_TYPE_COIL_READ,
    IVS_MSG_TYPE_COIL_WRITE,
} ivs_coil_msg_type_e;

typedef enum _operation_mode {
    OPERATION_NORMAL,
    OPERATION_REGAIN,
    OPERATION_OVERLOAD,
    OPERATION_OVERTEMP
} operation_mode_e;

typedef enum {
    LAFERT_VFD_AW1_MOTOR_THERMAL_PROTECTION = 1,
    LAFERT_VFD_AW1_CURRENT_SENSOR_OFFSET = 3,
    LAFERT_VFD_AW1_DC_BUS_OVERVOLTAGE = 4,
    LAFERT_VFD_AW1_DRIVE_OVER_TEMPERATURE = 5,
    LAFERT_VFD_AW1_DRIVE_OVERCURRENT = 6,
    LAFERT_VFD_AW1_DRIVE_PARAMETER_FAULT = 7,
    LAFERT_VFD_AW1_DC_BUS_UNDER_VOLTAGE = 12,
    LAFERT_VFD_AW1_EEPROM_FAULT = 14,
} lafert_vfd_alarm_word_1_e;

typedef enum {
    LAFERT_VFD_AW2_DSP_ADC_CALIBRATION = 0,
    LAFERT_VFD_AW2_MOTOR_OVERLOAD = 3,
    LAFERT_VFD_AW2_IGBT_OVER_TEMPERATURE = 4,
    LAFERT_VFD_AW2_UNDER_VELOCITY = 5,
    LAFERT_VFD_AW2_LOCKED_ROTOR = 8,
} lafert_vfd_alarm_word_2_e;

typedef enum {
    LAFERT_VFD_WW1_BIT_WARNING_PHASE_LOSS = 0,
} lafert_vfd_warning_word_1_e;
typedef enum {
    DANFOSS_VFD_AW1_BRAKE_CHECK_FAULT,
    DANFOSS_VFD_AW1_POWER_CARD_OVER_TEMPERATURE,
    DANFOSS_VFD_AW1_EARTH_FAULT,
    DANFOSS_VFD_AW1_CTRL_CARD_OVER_TEMPERATURE,
    DANFOSS_VFD_AW1_CTRL_WORD_TIMEOUT,
    DANFOSS_VFD_AW1_OVER_CURRENT,
    DANFOSS_VFD_AW1_TORQUE_LIMIT,
    DANFOSS_VFD_AW1_MOTOR_THERMISTOR_OVER_TEMPERATURE,
    DANFOSS_VFD_AW1_MOTOR_ETR_OVER_TEMPERTURE,
    DANFOSS_VFD_AW1_INVERTER_OVERLOADED,
    DANFOSS_VFD_AW1_DC_LINK_UNDER_VOLTAGE,
    DANFOSS_VFD_AW1_DC_LINK_OVER_VOLTAGE,
    DANFOSS_VFD_AW1_SHORT_CIRCUIT,
    DANFOSS_VFD_AW1_INRUSH_FAULT,
    DANFOSS_VFD_AW1_MAINS_PHASE_LOSS,
    DANFOSS_VFD_AW1_AMA_NOT_OK,
    DANFOSS_VFD_AW1_LIVE_ZERO_ERROR,
    DANFOSS_VFD_AW1_INTERNAL_FAULT,
    DANFOSS_VFD_AW1_BRAKE_OVERLOAD,
    DANFOSS_VFD_AW1_MOTOR_PHASE_U_IS_MISSING,
    DANFOSS_VFD_AW1_MOTOR_PHASE_V_IS_MISSING,
    DANFOSS_VFD_AW1_MOTOR_PHASE_W_IS_MISSING,
    DANFOSS_VFD_AW1_FIELDBUS_COMM_FAULT,
    DANFOSS_VFD_AW1_24V_SUPPLY_FAULT,
    DANFOSS_VFD_AW1_MAINS_FAILURE,
    DANFOSS_VFD_AW1_1POINT8_SUPPLY_FAULT,
    DANFOSS_VFD_AW1_BRAKE_RESISTOR_SHORT_CIRCUIT,
    DANFOSS_VFD_AW1_BRAKE_CHOPPER_FAULT,
    DANFOSS_VFD_AW1_OPTION_CHANGE,
    DANFOSS_VFD_AW1_DRIVE_INITIALIZED,
    DANFOSS_VFD_AW1_SAFE_STOP,
} danfoss_vfd_alarm_word_1_e;

typedef enum {
    DANFOSS_VFD_AW2_SERVICE_TRIP_RD_WR = 0,
    DANFOSS_VFD_AW2_SERVICE_TRIP_TYPECODE_SPAREPART = 2,
    DANFOSS_VFD_AW2_NO_FLOW = 5,
    DANFOSS_VFD_AW2_DRY_PUMP = 6,
    DANFOSS_VFD_AW2_END_OF_CURVE = 7,
    DANFOSS_VFD_AW2_BROKEN_BELT = 8,
    DANFOSS_VFD_AW2_FANS_ERROR = 18,
    DANFOSS_VFD_AW2_ECB_ERROR = 19,
} danfoss_vfd_alarm_word_2_e;

typedef enum {
    DANFOSS_VFD_WW1_BRAKE_CHECK_FAILED = 0,
    DANFOSS_VFD_WW1_POWER_CARD_OVER_TEMPERATURE,
    DANFOSS_VFD_WW1_EARTH_FAULT,
    DANFOSS_VFD_WW1_CTRL_CARD_OVER_TEMPERATURE,
    DANFOSS_VFD_WW1_CTRL_WORD_TIMEOUT,
    DANFOSS_VFD_WW1_OVER_CURRENT,
    DANFOSS_VFD_WW1_TORQUE_LIMIT,
    DANFOSS_VFD_WW1_MOTOR_THERMISTOR_OVER_TEMPERATURE,
    DANFOSS_VFD_WW1_MOTOR_ETR_OVER_TEMPERATURE,
    DANFOSS_VFD_WW1_INVERTER_OVERLOADED,
    DANFOSS_VFD_WW1_DC_LINK_UNDER_VOLTAGE,
    DANFOSS_VFD_WW1_DC_LINK_OVER_VOLTAGE,
    DANFOSS_VFD_WW1_DC_LINK_VOLTAGE_LOW,
    DANFOSS_VFD_WW1_DC_LINK_VOLTAGE_HIGH,
    DANFOSS_VFD_WW1_MAINS_PHASE_LOSS,
    DANFOSS_VFD_WW1_NO_MOTOR,
    DANFOSS_VFD_WW1_LIVE_ZERO_ERROR,
    DANFOSS_VFD_WW1_10V_LOW,
    DANFOSS_VFD_WW1_BRAKE_RESISTOR_POWER_LIMIT,
    DANFOSS_VFD_WW1_BRAKE_RESISTOR_SHORT_CIRCUIT,
    DANFOSS_VFD_WW1_BRAKE_CHOPPER_FAULT,
    DANFOSS_VFD_WW1_SPEED_LIMIT,
    DANFOSS_VFD_WW1_FIELDBUS_COMM_FAULT,
    DANFOSS_VFD_WW1_24V_SUPPLY_FAULT,
    DANFOSS_VFD_WW1_MAINS_FAILURE,
    DANFOSS_VFD_WW1_CURRENT_LIMIT,
    DANFOSS_VFD_WW1_LOW_TEMPERATURE,
    DANFOSS_VFD_WW1_VOLTAGE_LIMIT,
    DANFOSS_VFD_WW1_ENCODER_LOSS,
    DANFOSS_VFD_WW1_OUTPUT_FREQUENCY_LIMIT,
} danfoss_vfd_warning_word_1_e;

typedef enum {
    DANFOSS_VFD_WW2_START_DELAYED = 0,
    DANFOSS_VFD_WW2_STOP_DELAYED = 1,
    DANFOSS_VFD_WW2_CLOCK_FAILURE = 2,
    DANFOSS_VFD_WW2_NO_FLOW = 5,
    DANFOSS_VFD_WW2_DRY_PUMP = 6,
    DANFOSS_VFD_WW2_END_OF_CURVE = 7,
    DANFOSS_VFD_WW2_BROKEN_BELT = 8,
    DANFOSS_VFD_WW2_FANS_WARNING = 18,
    DANFOSS_VFD_WW2_ECB_WARNING = 19,
} danfoss_vfd_warning_word_2_e;

typedef enum {
    SYS_ALARM_VFD_OVER_TEMPERATURE,
    SYS_ALARM_VFD_OVER_CURRENT,
    SYS_ALARM_EXTERNAL_VFD_VOLTAGE,
    SYS_ALARM_INTERNAL_VFD_VOLTAGE,
    SYS_ALARM_INTERNAL_VFD,
    SYS_ALARM_VFD_PARAMETER,
    SYS_ALARM_VFD_STARTUP,
    SYS_ALARM_OTHER_VFD,
    SYS_ALARM_VFD_COMMUNICATION,
    SYS_ALARM_VFD_SPEED,
    SYS_ALARM_VSD_INIT_FAILURE,
    SYS_ALARM_COUNT,
} sys_alarm_e;

typedef enum {
    SYS_WARNING_VFD_OVER_TEMPERATURE,
    SYS_WARNING_VFD_OVER_CURRENT,
    SYS_WARNING_EXTERNAL_VFD_VOLTAGE,
    SYS_WARNING_INTERNAL_VFD_VOLTAGE,
    SYS_WARNING_INTERNAL_VFD,
    SYS_WARNING_RESERVED,
    SYS_WARNING_VFD_STARTUP,
    SYS_WARNING_OTHER_VFD,
    SYS_WARNING_VFD_COMMUNICATION,
    SYS_WARNING_VFD_SPEED,
    SYS_WARNING_VFD_WIRING,
    SYS_WARNING_SYSTEM_OVER_TEMPERATURE,
    SYS_WARNING_SYSTEM_UNDER_TEMPERATURE,
    SYS_WARNING_BATTERY_UNDER_VOLTAGE,
    SYS_WARNING_BMS_COMM_LOSS,
    SYS_WARNING_VFD_COMM_LOSS,
    SYS_WARNING_VFD_PARAMETER_WRONG,
    SYS_WARNING_VFD_INIT_FAILURE,
    SYS_WARNING_VFD_SPEED_SET_FAILURE,
    SYS_WARNING_VFD_START_SET_FAILURE,
    SYS_WARNING_SENSORLESS_ERROR,
    SYS_WARNING_HAND_MODE_TIMEOUT,
    SYS_WARNING_EXTERNAL_INPUT_SETSPEED,
} sys_warning_e;

struct {
    //Trend Variables
    float trend_var_01;
    float trend_var_02;
    float trend_var_03;
    float trend_var_04;
    float trend_var_05;
    float trend_var_06;
    float trend_var_07;
    float trend_var_08;
    float trend_var_09;
    float trend_var_10;
    float trend_var_11;
    float trend_var_12;
    float trend_var_13;
    float trend_var_14;
    float trend_var_15;
    float trend_var_16;
    float trend_var_17;
    float trend_var_18;
    float trend_var_19;
    float trend_var_20;
    int trend_hours;

} rnd;
// alarm data structure
typedef struct alarm_tag {
    union {
        struct {
            //vfd alarm bits
            uint32_t vfd_over_temperature :1;     // over/under temperature
            uint32_t vfd_over_current :1;     // current related errors
            uint32_t ext_vfd_voltage :1;     // voltage error (over/under, etc.)
            uint32_t int_vfd_voltage :1; // internal voltage generated by vfd is in error
            uint32_t internal_vfd :1;     // general internal fault to the vfd
            uint32_t vfd_parameter :1;   // parameter fault generated by the vfd
            uint32_t vfd_startup :1;     //startup of vfd fault
            uint32_t other_vfd :1;  // other faults that may be generated by vfd
            uint32_t vfd_communication :1; // communication that the VFD is expecting to see is not there
            uint32_t vfd_speed :1;     // the speed is not correct
            uint32_t vfd_init_failure :1;    // initialization of the vfd failed

            //system alarm bits
            //uint32_t sys_over_temperature:        1;
            //uint32_t under_temperature:           1;
        } bits __attribute__((packed));
        uint32_t alarm_reg;
    } alarms;
    //uint32_t alarms_reset;
    uint32_t vfd_alarm_1;           // alarm register from VFD
    uint32_t vfd_alarm_2;           // alarm register from VFD

    union {
        struct {
            //vfd warning bits
            uint32_t vfd_over_temperature :1;       // over/under temperature
            uint32_t vfd_over_current :1;           // current related warnings
            uint32_t ext_vfd_voltage :1;   // voltage warning (over/under, etc.)
            uint32_t int_vfd_voltage :1; // internal voltage generated by vfd is in a warning state
            uint32_t internal_vfd :1; // general internal fault warning to the vfd
            uint32_t reserved00 :1;         // reserved
            uint32_t vfd_startup :1;            //startup of vfd warning
            uint32_t other_vfd :1; // other warnings that may be generated by vfd
            uint32_t vfd_communication :1; // communication that the VFD is expecting to see is not there
            uint32_t vfd_speed :1;          // the speed is not correct
            uint32_t vfd_wiring :1; // the wiring to the VFd has a fault on it
            //system warning bits
            uint32_t sys_over_temperature :1; // over temperature alarm for the control card
            uint32_t sys_under_temperature :1; // under temperature alarm for the control card
            uint32_t under_voltage_battery :1;      // RTC battery under voltage
            uint32_t bms_com_loss :1; // lost BMS communication for longer than timeout period (modbus or BACnet)
            uint32_t vfd_com_loss :1;   // communication with the vfd is lost
            uint32_t vfd_parameter_wrong :1; // communication with vfd has a wrong parameter
            uint32_t vfd_init_failure :1; // setting set during initialization failed
            uint32_t vfd_speed_set_failure :1; // the speed sent to the VFD is not the value read back
            uint32_t vfd_start_set_failure :1; // the start command sent to the VFD is no the value read back
            uint32_t hand_mode_enabled :1; // the pump has been in hand mode too long
            uint32_t ext_input_setspeed_enabled :1; // the pump speed is set by external signal
        } __attribute__((packed)) bits;
        uint32_t warning_reg;
    } warnings;
    uint32_t warnings_ack;
    uint32_t vfd_warning_1;     // warning register from VFD
    uint32_t vfd_warning_2;     // warning register from VFD
} alarm_t;
//-----------Global variables for IPS-------------------
typedef union {

    struct data_tag {
        float speed[IPS_NO_OF_PUMPS];
        uint16_t zone1_eng_units;
        uint16_t no_of_zones;
        uint16_t no_of_pumps;
        uint16_t active_zone;
        float active_zone_error;
        float active_zone_pv;
        float active_zone_sp;
        struct {
            float power;                // power (*10 W)
            float voltage;              // motor voltage (/10 V)
            float frequency;            // frequency (/10 Hz)
            float current;              // motor current (/100 A)
            float frequency_pct;        // frequency (/100 %)
            float torque;               // motor torque (/10 Nm)
            float temperature;          // motor temperature (/10 ?C)
            uint16_t speed;             // motor speed (rpm)
            uint16_t coil_speed;        // motor speed from coil reading
            //uint8_t motor_thermal;    // motor thermal (%)
            int16_t torque_pct;         // motor torque (%)
            float bus_voltage;          // bus voltage (V)
            /*new added */
            uint32_t ivs_read_data;         //store register value read from IVS
            int flow_controller_for_eeprom; //indicate out which step right now in the flow of value changing
            int flow_controller_for_eeprom_read;
            int flow_controller_for_eeprom_close;
            int ivs_parameter_setting; //delegates the whole complete vfd communication finished
            int ivs_coil_control; //indicate that this time communication is related with coil
            uint8_t receive_status;
            bool runstatus;
            pump_start_e start;
            /*new added */
            //int register_receive_indicator;
            int eeprom_error_in_open;
            int eeprom_error_in_close;
            uint32_t ivs_request_status;

            uint16_t ivs_request_address; //accept request from web interface and send to IVS
            /*new added */
            uint16_t ivs_register_number;    //how many registers are requested
            /*new added */
            //uint16_t coil_control_for_eeprom;
            //ivs_msg_type_e ivs_register_control_bit; //control whether reading from or writing in the IVS register
            ivs_coil_msg_type_e ivs_coil_control_bit; //indicate out whether reading or writing in the IVS coil
            //uint16_t eeprom_controllor;
            /*new added */
            uint32_t ivs_write_data; //store the value should be written in the IVS register
            /*new added */
            uint16_t ivs_response_indication; //indicate the response result and show the details on the web interface
            uint16_t motor_nominal_speed;       //the nominal speed of the motor
            uint16_t motor_rated_voltage;       //the rated voltage of the motor
            float motor_rated_current;      // the rated current of the motor
            float motor_rated_power;            // the rated power of the motor
            float tripping_current;             // iECM tripping current
            uint16_t overload_timer;    //iECM overload timer
//          uint16_t status1;
            union {
                struct {
                    uint32_t alarm :1;
                    uint32_t triplock_alarm :1;
                    uint32_t warning :1;
                    uint32_t over_current :1;
                    uint32_t over_temperature :1;
                } __attribute__((packed)) bits;
                uint32_t status_reg;
            } status;
            uint32_t ext_status_1;      // vfd status - danfoss
            uint32_t ext_status_2;      // vfd status - danfoss
            //uint8_t ident_fc_type[7]; // FC Type identification
            //uint8_t ident_power[21];  //
            //uint8_t ident_voltage[21];    //
            //uint8_t ident_sw_ver[6];  // vfd software version
            float software_rev; // The software revision version as read from modbus during init
            uint16_t hardware_rev; // The hardware revision version as read from modbus during init
            uint16_t parameter_rev;     // Lafert parameter revision number
            struct {
                uint16_t rotation_change :1; // when the flag is 1, rotation has changed, update the rotation
                uint16_t ramp_time_change :1; // when the flag is 1, ramp time has changed, update the ramp time
                uint16_t rotation_change_successful :1; //TODO: this bit is just set and cleared, not used
                uint16_t ramp_time_change_successful :1;
                uint16_t analog_in_cfg_change :1;
                uint16_t init_rotation_check :1;
                uint16_t switching_freq_change :1;
                uint16_t ivs_receive_select :1; //which receive callback to use for application
                uint16_t enter_ivs_request_mode :1; //indicate that there is an IVS request
                uint16_t relay_change :1; // when the flag is 1, relay setting has been changed
            } __attribute__ ((packed)) flags;
            struct {
                //TODO: remove analog/digital variables above
                uint16_t analog_input_1; // analog input 1 - settable: 4-20 or 0-10 (Danfoss input 53)
                uint16_t analog_input_2; // analog input 2 - settable: 4-20 or 0-10 (Danfoss input 54)
                uint16_t analog_output;     // analog output - 4-20mA
                uint8_t digital_inputs;     // digital inputs (2 bits used)
                uint8_t digital_outputs; // digital outputs (2 bits used) bit 0,1 is digital output 1,2, bit 4,5 is relay output 1,2
                //uint8_t relays;           // relay states (2 bits)
                uint8_t relay_params; // parameter needed to setup the relay function
                output_states output_state;
                uint8_t analog_input_setting; // the setting for analog input (bitfield for analog input settings) (0 = 4-20, 1 = 0-10)
                analog_in_config_e analog_in_1_cfg; // analog input 53 switch setting: [0] Current, [1] - Voltage for danfoss
                analog_in_config_e analog_in_2_cfg; // analog input 54 switch setting: [0] Current, [1] - Voltage for danfoss
            } io;
            uint8_t run_time;
            float bypass_status;
            float min_set_pressure;
        } vfd;
        struct {
            float actual_head; // pressure difference from input to output (0 - 999.9 ft)
            float actual_flow; // (0 - 99999.9 usGPM)
            uint8_t number; // 1 - 255
            pump_start_e status;
            uint16_t trip;
            uint16_t trip_kwh;
            uint16_t requested_speed; // speed requested by the controller to the VFD
            float error[10];
            float set_point;
            uint8_t bypass_valve; // bypass valve in percentage
            uint8_t vsd_get_data_delay; // in sys tick units
            bool dynamic_gain;
            float pid_gain;
            operation_mode_e op_mode;
            struct {
                float total_flow;       //
                float total_head;       //
                float total_power;      //
                uint8_t num_pumps_running;
                uint8_t num_pumps_connected;
                uint8_t ps_fallback_operation;
                struct {
                    pump_start_e canbus_start;
                    float canbus_setpoint;
                    //cal_mode_e canbus_setpoint_type;
                } slave;
                float set_point;
                float set_point1;
                uint16_t speed_set_point;
                bool flag_send_set_point;
                bool flag_send_set_speed;
                bool flag_alternation_in_progress;
                uint8_t alternation_in_progress_period; //max. of 255 seconds
            } parallel;
            struct {
                float sensor1_head;
                float sensor2_head;
                float sensor1_error[10];
                float sensor2_error[10];
                float speed1_error[10];
                float speed2_error[10];
                float error_percent;
            } inputs;
        } pump;
        struct {
            struct {
                uint16_t bms_set_speed;
                pump_start_e bms_start;
                hand_flag_t bms_hoa;
            } bms;
        } communication;
        struct {
            struct tm time;
            uint32_t iot_counter;
            struct {
                uint8_t result;
                float new_design_head;
                float new_design_flow;
                float new_zero_flow_head;
                uint16_t speed;
            } auto_flow_balance;
            struct {
                time_t pump_running_seconds;
                float pump_running_kwh;
                time_t controller_running_seconds;
                uint8_t vfd_reset_count[SYS_ALARM_COUNT];
                uint8_t vfd_reset_flag[SYS_ALARM_COUNT];
                uint32_t vfd_reset_timer;
            } counters;
            bool certificate_flag; //indicate whether certificate is installed or not
            union {
                struct {
                    uint8_t bms_control_active :1;
                    uint8_t display_logout_popup :1;
                    //uint8_t display_triplock_popup :1;
                    uint8_t vfd_alarm :1;
                    uint8_t vfd_triplock_alarm :1;
                } __attribute__ ((packed, aligned(1))) bits;
                uint8_t value;
            } indication;
        } system;
        alarm_t alarms; //alarms structure

        struct {
            float BTU_Per_Hour_112;
            float Total_BTU_Counter_281;
            float Trip_BTU_Counter_282;
            float Most_Open_Valve_Position_307;
            float Reset_Alarms_403;
            float Sensor_1_Setpoint_512;
            float Sensor_2_Setpoint_513;
            float Flow_Threshold_DO1_514;
            float Flow_Threshold_DO2_515;
            float Alternation_Interval_516;
            float Unbalance_for_Alternation_517;
            float Lead_Pump_518;
            float Sleeping_Time_519;
            float analog_output_1;
            bool bo1;
            bool bo2;
            uint16_t available_pump_03;
            uint16_t minimum_flow_override_04;
            uint16_t Maximum_flow_override_05;
            uint16_t current_limit_override_06;
            uint16_t High_temperature_override_07;
            uint16_t Flow_above_threshold_AV514_08;
            uint16_t Flow_above_threshold_AV515_09;
            uint16_t on_target_10;
            uint16_t Not_Reachable_11;
            uint16_t Not_In_Control_12;
            uint16_t Ramping_13;
        } undefined;
        float maxsuctionpress;
        float maxdischargepress;
        float speed1[IPS_NO_OF_PUMPS];

    } field;
    uint8_t byte[sizeof(struct data_tag)] __attribute__((packed));
} data_t;

struct {
    bool digital_in;
    bool digital_out;
} modbus_variable;

typedef union {
    struct {
        //Read coil global variables
        uint32_t pump1_hand_mode :1;
        uint32_t pump1_off_mode :1;
        uint32_t pump1_auto_mode :1;
        uint32_t pump2_hand_mode :1;
        uint32_t pump2_off_mode :1;
        uint32_t pump2_auto_mode :1;
        uint32_t pump3_hand_mode :1;
        uint32_t pump3_off_mode :1;
        uint32_t pump3_auto_mode :1;
        uint32_t pump1_run_feedback :1;
        uint32_t pump2_run_feedback :1;
        uint32_t pump3_run_feedback :1;
        uint32_t pump1_run_feedback_alarm :1;
        uint32_t pump2_run_feedback_alarm :1;
        uint32_t pump3_run_feedback_alarm :1;
    } bits __attribute__((packed));
} coils;

typedef union {
    struct app_config_field_tag {
        uint32_t check_sum;
        struct {
            uint16_t high_speed_limit;
        } vfd;

        uint8_t eeprom_pattern[4];
    } field;
    uint8_t byte[sizeof(struct app_config_field_tag)];
} app_configuration_t;

typedef enum {
    LANGUAGE_ENGLISH,
    LANGUAGE_FRENCH,
    LANGUAGE_SPANISH,
    LANGUAGE_CHINESE,
    LANGUAGE_TRADITIONAL_CHINESE,
    LANGUAGE_PORTUGUESE,
    LANGUAGE_LONG_TEXT,
    NUM_LANGUAGES
} language_e;

typedef enum {
    ORIENTATION_PORTRAIT,
    ORIENTATION_LANDSCAPE,
    ORIENTATION_PORTRAIT_FLIP,
    ORIENTATION_LANDSCAPE_FLIP,
    ORIENTATION_COUNT
} screen_orientation_e;
typedef enum {
    OFF_MODE,
    HAND_MODE,
    AUTO_MODE,
} hoamode_t;
typedef enum {
    LED_OFF,
    LED_BLINK,
    LED_ON,
} led_state_t;
typedef enum {
    BMS_SERIAL,
    BMS_TCP,
} bms_com_type_e;
typedef enum {
    STATIC_IP = 0,
    DHCP_IP = 1,
} ip_type_e;

typedef enum {
    WIFI_SEC_INVALID = 0, // Invalid security type.
    WIFI_SEC_OPEN, // WiFi network is not secured.
    WIFI_SEC_WPA_PSK, // WiFi network is secured with WPA/WPA2 personal(PSK).
    WIFI_SEC_WEP, //Security type WEP (40 or 104) OPEN OR SHARED.
    WIFI_SEC_802_1X //WiFi network is secured with WPA/WPA2 Enterprise.IEEE802.1x user-name/password authentication.
} m2m_sec_type_e;
typedef enum {
    WIFI_MODE_NONE,
    WIFI_MODE_ARM,
    WIFI_MODE_AP,
    WIFI_MODE_STA,
    WIFI_MODE_P2P,
    WIFI_MODE_CON,
} wifi_conn_mode_e;

typedef enum {
    MEM_TEST_STOP,
    MEM_TEST_START,
    MEM_TEST_FAIL,
    MEM_TEST_SUCCESS,
    MEM_TEST_RESET
} mem_test_state_e;
#define MAX_HOSTNAME_SIZE 32
#define MW_UART_NO_OF_PORT 3
#define MAX_IPV4_ADD_STRING_SIZE        16
// configuration data structure
//Note: if this changes then update the change_log array in mw_datalogging.c
typedef union {
    struct bc_config_field_tag {
        uint32_t check_sum;
        struct {
            long local_pass_1; // level 1 password
            long local_pass_2; // level 2 password
            long local_pass_3; // level 3 password
            long local_pass_4; // level 4 password
            uint8_t wifi_password[24]; // wifi password for the module
            uint8_t web_pass_1[24]; // web level 1 password
            uint8_t web_pass_2[24]; // web level 2 password
            uint8_t web_pass_3[24]; // web level 3 password
            uint8_t web_pass_4[24]; // web level 4 password
            uint8_t web_login_attempts; // number of unsuccessful times the web page can be logged in before it locks out any additional logins until it is reset on the LCD
            uint8_t flags; //security bit flags
            uint32_t authentication_timer; // timer for how long the login timeout is
        } security;
        struct {
            uint8_t factory_setup_flag :1; // initial support for factory setup
            uint8_t debug_mode_flag :1; // for internal debugging support
            uint8_t commissioning_flag :1; // commissioning flag
            uint8_t language_change_flag :1; // update language selected
            uint8_t counters_reset_flag :1;
            uint8_t local_hoa_active :1; //local stop/hand from interface
        } __attribute__((packed, aligned(1))) indication;
        struct {
            struct {
                uint8_t enabled; // time based logging enabled
                uint32_t time; // in seconds
                uint16_t lcd_trend_interval; //in hours
            } timer;
        } logs;
        struct {
            uint16_t activation_services;
            bool Cert_progress_triggered;
        } system;
        struct {
            uint8_t pump_serial_number[MAX_IPV4_ADD_STRING_SIZE + 1]; //pump serial string,
            uint8_t tag[MAX_IPV4_ADD_STRING_SIZE + 1]; // pump tag identifier string (used for hostname, ssid)
        } identification;
        struct {
            uint8_t modbus_id; // modbus ID
            hoamode_t hoa_mode;
        } vfd;
        struct {
            uint8_t backlight_level; // backlight level
            screen_orientation_e orientation; // orientation of the display
        } lcd;
        led_state_t system_leds[NUM_LEDS]; // array of leds containing the state as well
        struct {
            date_format_e date_format; // date format (MM/DD/YY, DD/MM/YY)
            time_format_e time_format; // time format (AM/PM, 24hr)
            temperature_format_e temperature; // temperature unit (C, F)
            pressure_format_e pressure; // pressure unit (BAR, kPa, PSI, ft, m)
            flow_format_e flow; // flow unit (l/s, m3/h, GPM)
            language_e language; // langugage (english, french, simplified Chinese, Spanish, Portuguese)
            speed_e speed; // speed (rpm, hz)
            power_format_e power; // power (kw, hp)
        } __attribute__((packed, aligned(1))) units;
        struct {
            struct {
                uint32_t baud_rate[MW_UART_NO_OF_PORT]; // communication speed (standard baud rates)
                uint32_t com_config[MW_UART_NO_OF_PORT]; // configuration setting, defines from mw (data bits, stop bits, parity, etc)
                uint16_t com_timeout[MW_UART_NO_OF_PORT]; // com timeout waiting for to receive bits
                uint16_t com_recv_timeout[MW_UART_NO_OF_PORT]; // communication timeout after receiving some bits.
            } serial_port;
            struct {
                uint16_t version;
                uint8_t communication_start_delay; //max. of 255 seconds
                struct {
                    uint8_t enable; //enable modbus slave
                    uint8_t enable_master; //enable modbus TCP/IP master   Added by kiran hatti
                    uint8_t address; // depc address on the BMS Modbus network
                    bms_com_type_e type; // is this through serial or ethernet
                    uint8_t remote_ip_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // IP address
                    int no_of_slaves;
                    bool master_disable;
                    char slave_1_ip_address[16 + 1]; // Slave 1 IP address
                    char slave_2_ip_address[16 + 1]; // Slave 2 IP address
                    char slave_3_ip_address[16 + 1]; // Slave 3 IP address
                    char slave_4_ip_address[16 + 1]; // Slave 4 IP address
                    //bms_interface_type interface_type;
                } modbus;
                struct {
                    //not used by embitel
                    /*
                     uint8_t enable; // enable BACnet
                     uint32_t device_instance;   // BACnet device instance number
                     uint8_t mac_address;
                     uint8_t max_info_frames;
                     uint8_t max_master;
                     uint8_t *dev_name;
                     bms_com_type_e type;    // is this through serial or ethernet
                     uint8_t location[25];
                     uint8_t description[25];
                     */

                    //for removing error
                    uint8_t enable; // enable BACnet
                    uint16_t port; // IP port
                    bool foreign_device;
                    char fd_ip_address[MAX_IPV4_ADD_STRING_SIZE + 1];
                    uint16_t fd_port; // IP port for Foreign device
                    uint16_t ttl; //Time to live in seconds
                    uint32_t device_instance; // BACnet device instance number
                    //uint32_t instance_num;      // BACnet device instance number
                    uint8_t mac_address;
                    uint8_t max_info_frames;
                    uint8_t max_master;
                    uint8_t *dev_name;
                    bms_com_type_e type; // is this through serial or ethernet
                    uint8_t location[25];
                    uint8_t description[25];
                    uint32_t apdu_timeout;
                    uint32_t max_segments;
                    uint16_t apdu_retries;

                } bacnet;
            } bms;
            struct {
                uint8_t *host_name; //[MAX_IPV4_ADD_STRING_SIZE + 1];                   // Ethernet host name
                ip_type_e ip_mode; // DHCP/static IP
                uint8_t ip_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // IP address
                uint8_t gateway_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // gateway address
                uint8_t subnet_mask[MAX_IPV4_ADD_STRING_SIZE + 1]; // subnet mask
                uint8_t primary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // primary DNS server
                uint8_t secondary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // secondary DNS server
            } ethernet;
            struct {
                wifi_conn_mode_e mode; // mode of the wifi module (STA, AP)
                // AP mode settings
                uint8_t *ap_ssid; //[MAX_IPV4_ADD_STRING_SIZE + 1];                      // AP SSID
                m2m_sec_type_e ap_auth; // AP authentication mode
                uint8_t ap_password[MAX_HOSTNAME_SIZE + 1]; // AT mode password
                uint8_t ap_channel; // wifi channel number
                uint8_t ap_ip_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // IP address
                uint8_t ap_gateway_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // gateway address
                uint8_t ap_subnet_mask[MAX_IPV4_ADD_STRING_SIZE + 1]; // subnet mask
                uint8_t ap_primary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // primary DNS server
                uint8_t ap_secondary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // secondary DNS server
                uint8_t ap_min_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // minimum ip address range
                uint8_t ap_max_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // maximum ip address range
                //STA mode settings
                ip_type_e sta_ip_mode; //static or dynamic ip
                uint8_t sta_ssid[MAX_HOSTNAME_SIZE + 1]; // STA SSID
                m2m_sec_type_e sta_auth; // authentication type used
                uint8_t sta_password[MAX_HOSTNAME_SIZE + 1]; // STA mode password
                uint8_t sta_host_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // IP address
                uint8_t sta_gateway_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // gateway address
                uint8_t sta_subnet_mask[MAX_IPV4_ADD_STRING_SIZE + 1]; // subnet mask
                uint8_t sta_primary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // primary DNS server
                uint8_t sta_secondary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // secondary DNS server
                //ARM mode settings
                uint8_t arm_ssid[MAX_HOSTNAME_SIZE + 1]; // ARM SSID
                m2m_sec_type_e arm_auth; // authentication type used
                uint8_t arm_password[MAX_HOSTNAME_SIZE + 1]; // ARM mode password
                uint8_t arm_host_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // IP address
                uint8_t arm_gateway_address[MAX_IPV4_ADD_STRING_SIZE + 1]; // gateway address
                uint8_t arm_subnet_mask[MAX_IPV4_ADD_STRING_SIZE + 1]; // subnet mask
                uint8_t arm_primary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // primary DNS server
                uint8_t arm_secondary_dns[MAX_IPV4_ADD_STRING_SIZE + 1]; // secondary DNS server
                // P2P mode settings
                uint8_t *p2p_ssid; //[MAX_IPV4_ADD_STRING_SIZE + 1];         // destination SSID
                uint8_t p2p_channel; // channel number
            } wifi;
            struct {
                uint8_t id; // id of the controller, this is setup uniquely in the completed system
            } inter_card;
        } communication;
        struct {
            uint8_t save_to_db_tmr_interval; //max. of 255 minutes
        } backup;
        struct {
            uint32_t over_temperature :1; // over temperature alarm for the control card
            uint32_t under_temperature :1; // under temperature alarm for the control card
            uint32_t under_voltage_battery :1; // RTC battery under voltage
            uint32_t bms_comm_loss :1;
        } warning_enable; //either enabled or disabled
        struct {
            float over_temperature; // over temperature limit (deg C)
            float under_temperature; // under temperature limit (deg C)
            uint16_t battery_under_voltage; // battery under voltage limit for low RTCbattery (in mV)
            uint16_t bms_com_loss_timeout; // timeout period before the BMS communication loss is determined
            //            uint16_t vfd_com_loss_timeout;  // timeout period before the vfd communication loss is determined
        } warning_limits;
        struct {
            uint8_t bms_com_loss_mode; // what to do after BMS looses communication
        } alarm_actions;
        struct {
            uint8_t screen_display_test_mode; //turn on the test mode, and all alarm & warning will be displayed
            mem_test_state_e mem_test_mode;
        } test_flags;
        struct {
            uint16_t sub_sector_counts; //make it a variable for testing
        } test_params;
        uint8_t end_of_eeprom_test_pattern[4];
    } field;
    uint8_t byte[sizeof(struct bc_config_field_tag)];
} bc_configuration_t;
typedef union {
    struct config_field_tag {
        uint32_t check_sum;
        struct {
            float design_flow_mode[MAX_NO_OF_PARA_SET]; //parameters for dual season mode, 0 for mode1, 1 for mode2
            float design_head_mode[MAX_NO_OF_PARA_SET];
            float zero_flow_head_mode[MAX_NO_OF_PARA_SET];
            float min_flow_mode[MAX_NO_OF_PARA_SET];
            dualsea_mode_e dualsea_active_mode; //0=standard, 1=mode 1(heating mode), 2=mode 2(cooling mode)
            struct {
                uint8_t auto_flow_balance :1;
                uint8_t energy_perf_bundle :1;
                uint8_t protection_bundle :1;
                uint8_t zone_opt_bundle :1;
                uint8_t pump_manager :1;
            } __attribute__ ((packed)) service_activation;
        } system;
        struct {
            vfd_mfg_e manufacturer; // vfd manufacturer enum
            uint16_t motor_high_speed;  // highest motor operating speed
            uint16_t high_speed_limit; // the limit for the highest operating pump speed
            uint16_t low_speed_limit;   // lowest operating speed
            uint8_t num_poles;  // number of motor poles
            uint8_t modbus_id;  // modbus ID
            hand_flag_t hoa_mode;
            uint16_t ramp_up_time;  // the ramp up time of the vfd in seconds
            uint16_t ramp_down_time; // the ramp downp time of the vfd in seconds
            motor_rotation_e motor_rotation; // the rotation of the motor [0] clockwise [1]counter clockwise (Top View);
            switching_freq_e switching_freq_mode; // the switching frequency of the Danfoss motor, default to LOW_NOISE_MODE
            struct {
                analog_in_config_e analog_in_1_cfg; // analog input 1 switch setting: [0] Current, [1] - Voltage
                analog_in_config_e analog_in_2_cfg; // analog input 54 switch setting: [0] Current, [1] - Voltage
                uint8_t relays[2];
                struct {
                    uint8_t analog :1;
                    uint8_t digital :1;
                } __attribute__((packed)) outputs;
            } io;
        } vfd;
        struct { // this structure describes the pump operating points
            pump_start_e start; // sets whether the pump is started or stopped
            float design_flow;
            float design_head;
            float design_speed;
            float design_efficiency;
            float zero_flow_head;
            float avg_load_efficiency;
            float flow_threshold; //used for digital output
            float head_threshold; //used for digital output
            ai_sensors_control_e sensors_control_mode;
            float set_point_zone1; //used for analog input pressure control
            float set_point_zone2; //used for analog input pressure control
            float min_flow;
            float set_point;
            uint16_t skip_speed_lower_range[SKIP_SPEED_COUNT];
            uint16_t skip_speed_upper_range[SKIP_SPEED_COUNT];
            pump_startup_e startup_state; // start up operation of the pump
            pump_auto_mode_e mode; // mode of the pump
            pump_parallel_mode_e parallel_mode; // mode of the pump
            pump_control_mode_e control_mode; // control mode of the pump
            di_pump_control_e di1_control; // input 1 representation of digital value
            di_pump_control_e di2_control; // input 2 representation of digital value
            do_pump_control_e do1_control; // digital output 1 representation
            do_pump_control_e do2_control; // digital output 2 representation
            ro_pump_control_e ro1_control; // relay output 1
            ro_pump_control_e ro2_control; // relay output 2
            ai_pump_control_e ai1_control; // input  representation of the analog value
            ai_pump_control_e ai2_control; // analog 2 input representation
            ao_pump_control_e ao_control; // analog output representation
            manual_pump_control_e manual_control; // manual control - start or stop
            uint16_t di1_control_setspeed;   //set speed for digital input 1/18
            uint16_t di2_control_setspeed;   //set speed for digital input 2/19
            struct {
                float sensor1_high_ref;
                float sensor1_low_ref;
                float sensor1_high_pressure;
                float sensor1_low_pressure;
                float sensor2_high_ref;
                float sensor2_low_ref;
                float sensor2_high_pressure;
                float sensor2_low_pressure;
            } sensors;
            struct {
                float low_threshold;
                float high_threshold;
                uint16_t default_valve_position;    //in %
            } bypass_valve;
            struct {
                float k_p;
                float k_i;
                float k_d;
                float inputs_p;
                float inputs_i;
                float inputs_d;
                uint8_t gain_1;
                uint8_t gain_2;
                uint8_t gain_3;
                uint8_t gain_4;
            } pid;
            struct {
                uint8_t get_data_default_delay; //max. of 255 sys-ticks (51 seconds)
                uint8_t get_data_constant_flow_delay; //max. of 255 sys-ticks (51 seconds)
                uint8_t delay_between_two_frames; //max. of 255 fast-sys-ticks (510 ms)
            } vsd_comm;
            struct {
                hand_flag_t flags;
                uint16_t speed;
            } hand_mode;
            struct {
                uint8_t num_pumps; // number of pumps the controller is connected to
                float total_design_flow; //of the entire system, needs to be set on each
                float total_min_flow;
                float pump_head_bep;
                float pump_flow_bep;
                float dead_band;
                uint16_t fall_back_pct_max_speed;
                uint16_t alternation_interval; // in minutes, 0 for no alternation
                uint16_t min_on_time;               // in seconds
                uint8_t max_operating_count; //maximum number of pump to run in parallel
            } parallel_pump;
        } pump;
        struct {
            uint8_t bms_com_loss_mode; // what to do after BMS looses communication
            uint8_t vfd_reset_count_limit[SYS_ALARM_COUNT]; //there is fixed times for vfd hpi reset due to different alarms
            uint32_t vfd_reset_delay[SYS_ALARM_COUNT]; //there is fixed delay for vfd hpi reset due to different alarms
        } alarm_actions;
        struct {
            uint16_t vfd_com_loss_timeout; // timeout period before the vfd communication loss is determined
        } warning_limits;

        uint8_t eeprom_pattern[4];
    } field;
    uint8_t byte[sizeof(struct config_field_tag)];
} configuration_t;

typedef union {
    struct counters_field_tag {
        time_t pump_running_seconds_trip;
        time_t pump_running_seconds_odo;
        float pump_running_kwh_odo;
        float pump_running_kwh_trip;
        time_t controller_running_seconds_trip;
        time_t controller_start_time;
        time_t controller_running_seconds_odo;
        int32_t mem_test_time[5];
        uint8_t pattern_check[4];
    } field;
    uint8_t byte[sizeof(struct counters_field_tag)];
} counters_t;

typedef union {
    struct mfg_data_field_tag {
        uint8_t serial_number[14];
        uint16_t hw_version;
        time_t test_date;
        uint8_t model[4];
    } field;
    uint8_t byte[sizeof(struct mfg_data_field_tag)];
} mfg_data_t;

typedef enum {
    FLAG_MODBUS_WAIT_TCP,
    FLAG_MODBUS_WAIT_TCP1,
    FLAG_MODBUS_WAIT_TCP2,
    FLAG_MODBUS_WAIT_TCP3,
} txrxStatus_e;

//******************BMS Slave Communication variables***************

//-----------Global variables-------------------
typedef union {

    struct data_slave_var {
        bool bms_onoff;
        uint16_t speed;
        uint16_t setspeed;
        float Board_temperature;
    } field;
    uint8_t byte[sizeof(struct data_slave_var)];
} bms_slave_data_t;

//Global Retain Variables
typedef union {
    struct retain_data_var {
        uint32_t check_sum;
        hand_flag_t hoa_mode;
        int speed;
        uint8_t end_of_eeprom_global_pattern[4];
    } config;
    uint8_t byte[sizeof(struct retain_data_var)];
} glbconfig_t;

struct {
    bool dummy_alarm[14];
    bool dummy_alarmset[14];
    int alarm_count;
    int hourarray[14];
    int minarray[14];
    int dayarray[14];
    int montharray[14];
    int yeararray[14];
    bool ampmarray[14];
    int alarmstatus[14];
    bool alarmreset;

} BC_alarm;


float BMS_CountersHrs_Values[10];
uint16_t firmware_version[6];
uint16_t pump_frequency;
bool SendHoldReg;
bool write_bytes;
extern uint32_t timer_vfd_startup;
extern uint8_t vfd_com_loss_counter;
extern uint8_t di_in;
extern uint32_t drv_uart_unhandled_events_counter; //checking uart other not for 1.24
extern volatile glbconfig_t glbconfig;
extern volatile data_t data;
volatile configuration_t config;
volatile bc_configuration_t BC_config;
extern volatile app_configuration_t app_config;
extern volatile counters_t counters;
extern volatile mfg_data_t mfg_data;
extern volatile bms_slave_data_t bms_data;

extern uint8_t trend_graph_history_end[25];
extern uint8_t trend_graph_history_start[25];
extern uint8_t trend_history_end[25];
extern uint8_t trend_history_start[25];
extern uint8_t trend_history_type[10];

#endif /* TEST_FLAGS_H_ */

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
float ConvertTemperature(float value, conv_temperature_e convert_value);
float SourceConvertPressure(float value, conv_pressure_e convert_value);
float LoadConvertPressure(float value, conv_pressure_e convert_value);
float ConvertFlow(float value, conv_flow_e convert_value);
float ConvertPower(float value, conv_power_e convert_value);
float ConvertSpeed(float value, conv_speed_e convert_value);
float SourceConvertTemperature(float value, conv_temperature_e convert_value);
float LoadConvertTemperature(float value, conv_temperature_e convert_value);
//DictionaryString_e ConvertFlowString();
//DictionaryString_e ConvertPressureString();
//DictionaryString_e ConvertSpeedString();
//DictionaryString_e ConvertPowerString();
uint16_t ConvertFloatToUint16(
        float min,
        float max,
        float measuredValue,
        float manufacturer,
        float conversionfactor);
uint16_t ConvertRampTimeToRampParameter(float ramp_time);

float Unit_Conversion(convert_parameter Conversion_Parameter,float Value, units InputUnit, units OutputUnit);
#endif /* APPLICATION_IEEE754_H_ */

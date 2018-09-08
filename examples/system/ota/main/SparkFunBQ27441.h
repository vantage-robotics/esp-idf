/******************************************************************************
SparkFunBQ27441.h
BQ27441 Arduino Library Main Header File
Jim Lindblom @ SparkFun Electronics
May 9, 2016
https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Definition of the BQ27441 library, which implements all features of the
BQ27441 LiPo Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- SparkFun Battery Babysitter

Development environment specifics:
Arduino 1.6.7
SparkFun Battery Babysitter v1.0
Arduino Uno (any 'duino should do)
******************************************************************************/

#ifndef SparkFunBQ27441_h
#define SparkFunBQ27441_h

#include "BQ27441_Definitions.h"

#define BQ72441_I2C_TIMEOUT 2000

// Parameters for the current() function, to specify which current to read
typedef enum {
	AVG,  // Average Current (DEFAULT)
	STBY, // Standby Current
	MAX   // Max Current
} current_measure;

// Parameters for the capacity() function, to specify which capacity to read
typedef enum {
	REMAIN,     // Remaining Capacity (DEFAULT)
	FULL,       // Full Capacity
	AVAIL,      // Available Capacity
	AVAIL_FULL, // Full Available Capacity
	REMAIN_F,   // Remaining Capacity Filtered
	REMAIN_UF,  // Remaining Capacity Unfiltered
	FULL_F,     // Full Capacity Filtered
	FULL_UF,    // Full Capacity Unfiltered
	DESIGN      // Design Capacity
} capacity_measure;

// Parameters for the soc() function
typedef enum {
	FILTERED,  // State of Charge Filtered (DEFAULT)
	UNFILTERED // State of Charge Unfiltered
} soc_measure;

// Parameters for the soh() function
typedef enum {
	PERCENT,  // State of Health Percentage (DEFAULT)
	SOH_STAT  // State of Health Status Bits
} soh_measure;

// Parameters for the temperature() function
typedef enum {
	BATTERY,      // Battery Temperature (DEFAULT)
	INTERNAL_TEMP // Internal IC Temperature
} temp_measure;

// Parameters for the setGPOUTFunction() funciton
typedef enum {
	SOC_INT, // Set GPOUT to SOC_INT functionality
	BAT_LOW  // Set GPOUT to BAT_LOW functionality
} gpout_function;


	//////////////////////////////
	// Initialization Functions //
	//////////////////////////////
	/**
	    Initializes class variables
	*/

	
	/**
	    Initializes I2C and verifies communication with the BQ27441.
		Must be called before using any other functions.
		
		@return true if communication was successful.
	*/
	bool gaugeBegin(void);
	
	/**
	    Configures the design capacity of the connected battery.
		
		@param capacity of battery (unsigned 16-bit value)
		@return true if capacity successfully set.
	*/
	bool setCapacity(uint16_t capacity);
	
	/////////////////////////////
	// Battery Characteristics //
	/////////////////////////////
	/**
	    Reads and returns the battery voltage
		
		@return battery voltage in mV
	*/
	uint16_t voltage(void);
	
	/**
	    Reads and returns the specified current measurement
		
		@param current_measure enum specifying current value to be read
		@return specified current measurement in mA. >0 indicates charging.
	*/
	int16_t current(current_measure type);
	
	/**
	    Reads and returns the specified capacity measurement
		
		@param capacity_measure enum specifying capacity value to be read
		@return specified capacity measurement in mAh.
	*/
	uint16_t capacity(capacity_measure type);
	
	/**
	    Reads and returns measured average power
		
		@return average power in mAh. >0 indicates charging.
	*/
	int16_t power(void);
	
	/**
	    Reads and returns specified state of charge measurement
		
		@param soc_measure enum specifying filtered or unfiltered measurement
		@return specified state of charge measurement in %
	*/
	uint16_t soc(soc_measure type);
	
	/**
	    Reads and returns specified state of health measurement
		
		@param soh_measure enum specifying filtered or unfiltered measurement
		@return specified state of health measurement in %, or status bits
	*/
	uint8_t soh(soh_measure type);
	
	/**
	    Reads and returns specified temperature measurement
		
		@param temp_measure enum specifying internal or battery measurement
		@return specified temperature measurement in degrees C
	*/
	uint16_t temperature(temp_measure type);
	
	////////////////////////////	
	// GPOUT Control Commands //
	////////////////////////////
	/**
	    Get GPOUT polarity setting (active-high or active-low)
		
		@return true if active-high, false if active-low
	*/
	bool GPOUTPolarity(void);
	
	/**
	    Set GPOUT polarity to active-high or active-low
		
		@param activeHigh is true if active-high, false if active-low
		@return true on success
	*/
	bool setGPOUTPolarity(bool activeHigh);
	
	/**
	    Get GPOUT function (BAT_LOW or SOC_INT)
		
		@return true if BAT_LOW or false if SOC_INT
	*/
	bool GPOUTFunction(void);
	
	/**
	    Set GPOUT function to BAT_LOW or SOC_INT
		
		@param function should be either BAT_LOW or SOC_INT
		@return true on success
	*/
	bool setGPOUTFunction(gpout_function function);
	
	/**
	    Get SOC1_Set Threshold - threshold to set the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t SOC1SetThreshold(void);
	
	/**
	    Get SOC1_Clear Threshold - threshold to clear the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t SOC1ClearThreshold(void);
	
	/**
	    Set the SOC1 set and clear thresholds to a percentage
		
		@param set and clear percentages between 0 and 100. clear > set.
		@return true on success
	*/
	bool setSOC1Thresholds(uint8_t set, uint8_t clear);
	
	/**
	    Get SOCF_Set Threshold - threshold to set the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t SOCFSetThreshold(void);
	
	/**
	    Get SOCF_Clear Threshold - threshold to clear the alert flag
		
		@return state of charge value between 0 and 100%
	*/
	uint8_t SOCFClearThreshold(void);
	
	/**
	    Set the SOCF set and clear thresholds to a percentage
		
		@param set and clear percentages between 0 and 100. clear > set.
		@return true on success
	*/
	bool setSOCFThresholds(uint8_t set, uint8_t clear);
	
	/**
	    Check if the SOC1 flag is set in flags()
		
		@return true if flag is set
	*/
	bool socFlag(void);
	
	/**
	    Check if the SOCF flag is set in flags()
		
		@return true if flag is set
	*/
	bool socfFlag(void);
	
	/**
	    Get the SOC_INT interval delta
		
		@return interval percentage value between 1 and 100
	*/
	uint8_t sociDelta(void);
	
	/**
	    Set the SOC_INT interval delta to a value between 1 and 100
		
		@param interval percentage value between 1 and 100
		@return true on success
	*/
	bool setSOCIDelta(uint8_t delta);
	
	/**
	    Pulse the GPOUT pin - must be in SOC_INT mode
		
		@return true on success
	*/
	bool pulseGPOUT(void);
	
	//////////////////////////
	// Control Sub-commands //
	//////////////////////////
	
	/**
	    Read the device type - should be 0x0421
		
		@return 16-bit value read from DEVICE_TYPE subcommand
	*/
	uint16_t deviceType(void);
	
	/**
	    Enter configuration mode - set userControl if calling from an Arduino
		sketch and you want control over when to exitConfig.
		
		@param userControl is true if the Arduino sketch is handling entering 
		and exiting config mode (should be false in library calls).
		@return true on success
	*/
	bool enterConfig(bool userControl);
	
	/**
	    Exit configuration mode with the option to perform a resimulation
		
		@param resim is true if resimulation should be performed after exiting
		@return true on success
	*/
	bool exitConfig(bool resim);
	
	/**
	    Read the flags() command
		
		@return 16-bit representation of flags() command register
	*/
	uint16_t flags(void);
	
	/**
	    Read the CONTROL_STATUS subcommand of control()
		
		@return 16-bit representation of CONTROL_STATUS subcommand
	*/
	uint16_t status(void);


#endif

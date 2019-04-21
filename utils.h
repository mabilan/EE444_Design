/**************************************************************************************
 * utils.h
 *
 * Utility functions used for NFC Tag Project, including:
 *      - initialize_GPIO(): all GPIO pins to output/low for LPM considerations
 *      - initialize_Clocks(): sets MCLK / SMCLK to 8 MHz, enables FRAM
 *      - initialize_ADC(): configures ADC to handle temperature sensor conversions
 * Created on: Apr 15, 2019
 *      Author: Mike
 ***************************************************************************************/

#ifndef UTILS_H_
#define UTILS_H_


/*****************
 * File Includes
 *****************/

#include "gpio.h"
#include "pmm.h"
#include "cs.h"
#include "adc.h"
#include "rtc.h"


/**************
 * Structures
 **************/

typedef struct Hour_Timer_type {
    uint16_t seconds;
    uint16_t minutes;
    uint16_t hours;
} Hour_Timer;


/**************
 *  Constants
 **************/

#define ADC_Base_Address    0x0700
#define RTC_Base_Address    0x0300
#define I2C_TX_BUFF_SIZE    64
#define SLAVE_ADDR          0x53
#define P4_SDA_PIN          BIT6
#define P4_SCL_PIN          BIT7


/***********************
 * Function Prototypes
 ***********************/

// Initializes all GPIO pins to low output
// Unlocks FRAM
inline void initialize_GPIO (void);

// Configures clocks for 8 MHz operation, according to
// User Guide procedure
inline void initialize_Clocks (void);

// Configures ADC for sampling on-board temperature sensor
// at a rate of 1 sample/second
inline void initialize_ADC (void);

// Configures RTC counter to alarm every 1 second
// RTC source is the VLOCLOCK: this provides reduced power
// consumption at the cost of increased drift (up to 50%).
inline void initialize_RTC(void);

//Configures the I2C registers for the MSP to operate in Master mode
//
inline void initialize_I2C(void);


#endif /* UTILS_H_ */

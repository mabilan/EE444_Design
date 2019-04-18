/**************************************************************************************
 *
 *  main.c
 *
 *  This program samples the MSP430FR2355's built-in temperature sensor every second,
 *  as indicated by the system's Real-Time Clock. Results are stored in a circular
 *  buffer, and transmitted via I2C upon request.
 *
 *
 **************************************************************************************/

#include <msp430.h> 
#include "utils.h"
#include "wdt_a.h"


/*******************************************
 *  Persistent (FRAM) Variable Declarations
 *******************************************/

#pragma PERSISTENT(RTC_time)
#pragma PERSISTENT(Temperature)
#pragma PERSISTENT(Temp_data)
#pragma PERSISTENT(head_index)
#pragma PERSISTENT(tail_index)
#pragma PERSISTENT(Temp_data)


/*********************
 * Constants/Defines
 ********************/

#define ADCREF_15V_30   *((unsigned int *)0x1A1A)       // Temp. Sensor Calibration for 30C
#define ADCREF_15V_105  *((unsigned int *)0x1A1C)       // Temp. Sensor Calibration for 105C
#define ADCREF_DIFF     (ADCREF_15V_105 - ADCREF_15V_30)// Difference in ADC ref values
#define BUFFERSIZE      28000                           // # of bytes for data storage

/************************
 * Variable Definitions
 ************************/

struct Hour_Timer_type RTC_time = {0, 0, 0};
uint8_t Temperature=0;
uint8_t Temp_data[BUFFERSIZE] = {0};
uint16_t head_index=0;
uint16_t tail_index=0;
uint8_t full_buffer=0;

/***************
 *  main.c
 ***************/

int main (void)
{

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
	initialize_GPIO ();
	initialize_Clocks ();
	initialize_ADC ();
	initialize_RTC ();
	PMM_enableInternalReference ();
	PMM_enableTempSensor ();

	// initialize_I2C();
	// other init functions Kyle needs
	RTC_start (RTC_Base_Address,
	           RTC_CLOCKSOURCE_XT1CLK);
	// Enter LPM0 with Global Interrupts Enabled
	__bis_SR_register (LPM3_bits + GIE);
}



/*******************************
 * Interrupt Service Routines
 *******************************/

// ISR for RTC interrupts
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR (void)
{
    // Increment RTC time counters
    ++(RTC_time.seconds);
    if (RTC_time.seconds == 60)
    {
        RTC_time.seconds = 0;
        ++(RTC_time.minutes);
        if (RTC_time.minutes == 60)
        {
            RTC_time.minutes = 0;
            ++(RTC_time.hours);
        }
    }

    // Sample temperature
    ADCCTL0 |= ADCENC | ADCSC;

    // Clear RTC interrupt flag
    RTCCTL &= ~RTCIFG;
}

// ISR for ADC interrupt
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR (void)
{
    // Disable additional ADC conversions
    ADCCTL0 &= ~ADCENC;

    Temperature = (75*16*(ADCMEM0 - ADCREF_15V_30))/(16*ADCREF_DIFF) + 30;

    if(full_buffer)
    {
        Temp_data[head_index] = Temperature;
        ++head_index;
        ++tail_index;
    }
    else
    {
        Temp_data[head_index] = Temperature;
        ++head_index;
        if (head_index == BUFFERSIZE-1)
        {
            head_index = 0;
            ++tail_index;
            full_buffer = 1;
        }

    }

    ADCIFG = 0;
}

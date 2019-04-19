/*
 * utils.c
 *
 *  Created on: Apr 15, 2019
 *      Author: Mike
 */

#include "utils.h"

/***************************
 * Function Definitions
 ***************************/
inline void initialize_GPIO (void)
{
    // Initialize all GPIO to output low for minimal LPM power consumption
    GPIO_setAsOutputPin (GPIO_PORT_PA, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin (GPIO_PORT_PB, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin (GPIO_PORT_PC, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin (GPIO_PORT_PD, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin (GPIO_PORT_PE, GPIO_PIN_ALL16);
    GPIO_setAsOutputPin (GPIO_PORT_PJ, GPIO_PIN_ALL16);

    GPIO_setOutputLowOnPin (GPIO_PORT_PA, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin (GPIO_PORT_PB, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin (GPIO_PORT_PC, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin (GPIO_PORT_PD, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin (GPIO_PORT_PE, GPIO_PIN_ALL16);
    GPIO_setOutputLowOnPin (GPIO_PORT_PJ, GPIO_PIN_ALL16);

    // Initialize GPIO 2.3 (Button) for input
    //Set P2.3 (S3) to input
    GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P2, GPIO_PIN3);
    GPIO_selectInterruptEdge (GPIO_PORT_P2,
                              GPIO_PIN3,
                              GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt (GPIO_PORT_P2, GPIO_PIN3);
    GPIO_clearInterrupt (GPIO_PORT_P2, GPIO_PIN3);

    // FRAM device unlock (prevents compiler warning)
    PMM_unlockLPM5 ();
}

inline void initialize_Clocks (void)
{
    // Configure FRAM to 0 wait-states for operation at 8 MHz
    // Must be done _before_ clock configuration
    FRCTL0 = FRCTLPW | NWAITS_0;

    P2SEL1 |= BIT6 | BIT7;                       // P2.6~P2.7: crystal pins
    do
    {
       CSCTL7 &= ~(XT1OFFG | DCOFFG);           // Clear XT1 and DCO fault flag
       SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag

    __bis_SR_register(SCG0);                     // disable FLL
    CSCTL3 |= SELREF__XT1CLK;                    // Set XT1 as FLL reference source
    CSCTL0 = 0;                                  // clear DCO and MOD registers
    CSCTL1 = DCORSEL_3;                          // Set DCO = 8MHz
    CSCTL2 = FLLD_0 + 244;                       // DCOCLKDIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                     // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));   // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;   // set XT1 (~32768Hz) as ACLK source, ACLK = 32768Hz
                                               // default DCOCLKDIV as MCLK and SMCLK source

}


inline void initialize_ADC (void)
{

    // Initialize ADC
    // SMCLK = 8 MHz (above)
    // No clock divider
    ADC_init (ADC_Base_Address,
              ADC_SAMPLEHOLDSOURCE_1,
              ADC_CLOCKSOURCE_SMCLK,
              ADC_CLOCKDIVIDER_1);

    // Temperature sensor requires 30 us sample time, ADC requires 14 conversion cycles
    // (30 us)/(8 MHz) = 240 cycles + 13 conversion cycles = 253 cycles -> 256 cycles for SHT
    // Multi-sampling disabled
    ADC_setupSamplingTimer (ADC_Base_Address,
                            ADC_CYCLEHOLD_256_CYCLES,
                            ADC_MULTIPLESAMPLESDISABLE);

    // Configures ADC Memory to store conversions from temperature sensor
    // Sets default voltage references
    ADC_configureMemory (ADC_Base_Address,
                         ADC_INPUT_TEMPSENSOR,
                         ADC_VREFPOS_INT,
                         ADC_VREFNEG_AVSS);

    // Turn ADC on
    ADC_enable (ADC_Base_Address);

    // Enable ADC interrupts for completed conversions
    ADC_enableInterrupt (ADC_Base_Address,
                        ADC_COMPLETED_INTERRUPT);
}

inline void initialize_RTC (void)
{
    // Halt RTC, if previously initialized
    RTC_stop (RTC_Base_Address);

    // Clear RTC Interrupt flags
    RTC_clearInterrupt(RTC_Base_Address,
                       RTC_OVERFLOW_INTERRUPT_FLAG);

    // RTCMOD = 32
    // Clock Divider = 1024
    // When RTC clock source is ACLK = 32768, every 32 cycles is one second
    RTC_init (RTC_Base_Address,
             32,
             RTC_CLOCKPREDIVIDER_1024);

    // Enable RTC interrupts for modulo counter overflow
    RTC_enableInterrupt (RTC_Base_Address,
                        RTC_OVERFLOW_INTERRUPT);

}

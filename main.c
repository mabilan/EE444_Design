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


/**********************************************
 * Hardware Platform Defines
 **********************************************
 *
 * The define aliases below are used to
 * indicate which ST25 NFC tag the device is
 * connected to. This setting will impact the
 * size of data number of 256 byte data blocks
 * that must be sent to NFC memory, based on
 * individual device capacity.
 * 
 * Set define value of target device to 1, set
 * remaining device define values to 0.
 *
 * Device List
 * ------------
 * ST25DV04K - 512 byte memory
 * ST25DV16K - 2048 byte memory
 * ST25DV64K - 8192 byte memory
 *
 * *******************************************/

#define ST25DV04K   1
#define ST25DV16K   0
#define ST25DV64K   0

#if ST25DV04K
#define WRITECYCLES 16
#endif

#if ST25DV16K
#define WRITECYCLES 64
#endif

#if ST25DV64K
#define WRITECYCLES 256
#endif


/*********************
 * Constants/Defines
 ********************/

#define ADCREF_15V_30   *((unsigned int *)0x1A1A)       // Temp. Sensor Calibration for 30C
#define ADCREF_15V_105  *((unsigned int *)0x1A1C)       // Temp. Sensor Calibration for 105C
#define ADCREF_DIFF     (ADCREF_15V_105 - ADCREF_15V_30)// Difference in ADC ref values
#define BUFFERSIZE      2056                            // # of bytes for data storage
#define WRITEBUFF       256                             // multi-byte write limit for NFC tag
#define NFC_ADDR_MSB    0x01                            //MSB for the block to write to
#define NFC_ADDR_LSB    0x44                            //LSB for the block to write to
#define FILL 'x'
/*******************************************
 *  Persistent (FRAM) Variable Declarations
 *******************************************/

#pragma PERSISTENT(RTC_time)
#pragma PERSISTENT(Temp_data)
#pragma PERSISTENT(head_index)
#pragma PERSISTENT(tail_index)
#pragma PERSISTENT(full_buffer)


/************************
 * Variable Definitions
 ************************/

struct Hour_Timer_type RTC_time = {0, 0, 0};
uint8_t Temperature=0;
uint8_t Temp_data[BUFFERSIZE] = {0};
uint16_t head_index=0;
uint16_t tail_index=0;
uint8_t full_buffer=0;
volatile unsigned char TXData[66]= {NFC_ADDR_MSB,NFC_ADDR_LSB,FILL,FILL,FILL,FILL};

/***************
 *  main.c
 ***************/

int main (void)
{
    int j;
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    initialize_GPIO ();
    initialize_Clocks ();
    initialize_ADC ();
    initialize_RTC ();
    initialize_I2C();

    PMM_enableInternalReference ();
    PMM_enableTempSensor ();

    RTC_start (RTC_Base_Address,
               RTC_CLOCKSOURCE_XT1CLK);
    // Enter LPM3, enable global interrupts
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
        Temp_data[tail_index] = Temperature;
        ++head_index;
        ++tail_index;
    }
    else
    {
        Temp_data[tail_index] = Temperature;
        ++tail_index;
        if (tail_index == BUFFERSIZE-1)
        {
            tail_index = 0;
            ++head_index;
            full_buffer = 1;
        }

    }
    ADCIFG = 0;
}

// ISR for Port 2 (Button) Interrupt
#pragma vector = PORT2_VECTOR
__interrupt void Button_Press (void)
{
    static char j=0;
    static char k=0;
    int i;
    // Clear P2.3 interrupt flag
    GPIO_clearInterrupt (GPIO_PORT_P2, GPIO_PIN3);
    GPIO_disableInterrupt (GPIO_PORT_P2, GPIO_PIN3);

    // Disable RTC (no more sampling)
    RTC_stop(RTC_Base_Address);
    TXData[0]=(NFC_ADDR_MSB)*k;
    TXData[1]= (NFC_ADDR_LSB)*j;
    if(j==3&&k==1){
        j=0;
        k=0;
    }else if(j==3){
        j=0;
        k=1;
    }else{
        j++;
    }
    for (i=0; i<64; i++)
         {
             if(head_index>=BUFFERSIZE)
             {
                 head_index = 0;
             }
             if(head_index == tail_index)
             {
                 TXData[2+i] == 0;
                 continue;
             }
             TXData[2+i] = Temp_data[head_index];
             head_index++;
         }
    UCB1CTLW0 |= UCTR+UCTXSTT ;
    while(((UCB1IFG & UCTXIFG0)==0));

    UCB1IE |= UCTXIE0;              // transmit,stop interrupt enable
                                    //attempt to write 0x55 just for debugging the lines
    __bis_SR_register(GIE);
    while((UCB1IFG & UCTXSTP));
    GPIO_enableInterrupt (GPIO_PORT_P2, GPIO_PIN3);
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCIB1_ISR(void)
{
    static int i=0;


    switch(__even_in_range(UCB1IV,0x1e))
    {
    case 0x00: //No interrupts
        break;
    case 0x02: // ALIFG
        break;
    case 0x04: //NACKIFG
        UCB1CTL1|= UCTXSTT;
        break;
    case 0x06: //STTIFG
        break;
    case 0x08: //STPIFG
        break;
    case 0x0A: //RXIFG3
        break;
    case 0x0C: //TXIFG3
        break;
    case 0x0E: //RXIFG2
        break;
    case 0x10: //TXIFG2
        break;
    case 0x12: //RXIFG1
        break;
    case 0x14: //TXIFG1
        break;
    case 0x16: //RXIFG0
        break;
    case 0x18: //TXIFG0

        while(UCB1IFG & UCTXIFG0);
             UCB1TXBUF=TXData[i];
             while(UCB1IFG & UCTXIFG0);
        if(i==64)
        {
            UCB1IFG &= ~UCTXIFG;
            UCB1CTL1 |= UCTXSTP;
            i=0;
            break;
        }
        i++;
        break;
    case 0x1A: //BCNTIFG
        break;
    case 0x1C: //clock low timeout
        break;
    case 0x1E: //9th bit
        break;
    default:
        break;
    }
}

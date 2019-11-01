// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

#include "hw_uart.h"
#include "udma.h"
#include "gpio.h"
#include "hw_apps_rcm.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "udma_if.h"
#include "gpio_if.h"
#include "timer_if.h"


#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     2

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

volatile static tBoolean bRxDone;
volatile static tBoolean bTimerDone;
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
volatile float delay_up = 1000/48;
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
//! The interrupt handler for the timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int n=0;
void
TimerBaseIntHandler(void)
{
    //This interrupt is called when Timer_IF_IntSetup is called and this clears the interrupt
    Timer_IF_InterruptClear(g_ulBase);
    //This makes bTimerDone true
    bTimerDone = true;
}

void delay_function(float freq)
{
    // Load the time with the delay value which should be in ms.
    Timer_IF_Start(g_ulBase, TIMER_A, freq);
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{
    int k;
    long adc_value;
    long data[1100];
    int i;

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    while(1)
    {
        //reads 1050 samples of data, which is about 15 sec.
        for(i=0;i<1050;i++)
        {
            //
            // Send the string to slave. Chip Select(CS) needs to be
            // asserted at start of transfer and de-asserted at the end.
            //
            MAP_SPITransfer(GSPI_BASE,0,g_ucRxBuff,50,
                    SPI_CS_ENABLE|SPI_CS_DISABLE);

            // read the value from the ADC
            adc_value = ((int)(g_ucRxBuff[0])<<8 | g_ucRxBuff[1] );
            adc_value >>=3;
            adc_value = adc_value & 0x3FF;

            data[i]=adc_value;
            // Delay for maintaining the sample rate.
            delay_function(10);
            while(!bTimerDone);
                //Reset timer flag
                bTimerDone = false;
        }

        int state = 0;
        int prev_data = 0;
        int BPM = 0;

        // Computes the peak and outputs the BPM
        // state 1 is when we are on the rising edge.
        // state 2 detects the falling edge
        // when we again detect the rising edge, we count is as a pulse.
        // we eliminate the intermediate points, to get rid of the noise.

        for(i=0;i<1050;i= i + 4)
        {
            //printf("state %d data %d prev_data %d\n",state,data[i],prev_data);
            // Detect the rising edge.
            if (data[i] > prev_data & state == 0)
            {
                state = 1;
            }

            // detect the falling edge.
            else if (data [i] < prev_data & state == 1)
            {
                state = 2;
            }

            // detect the rising edge again.
            else if (data [i] > prev_data & state == 2)
            {
                state = 0;
                //printf("%d %d\n",data [i],prev_data);
                BPM++;
            }

            prev_data = data[i];
        }

        // multiply by 4 to get the heart rate.
        printf("\nHeart Rate: %d BPM\n",BPM*4);



}
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    bTimerDone = false;
    g_ulBase = TIMERA0_BASE;
    // Configuring the timer to one shot count down mode
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_ONE_SHOT, TIMER_A, 0);

    // Setup the interrupts for the timer timeout.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);



    MasterMain();

}


#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"

#include "utils/uartstdio.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

static volatile uint32_t g_ui32ADC0Reading = 9999;

//*****************************************************************************
//
// Counters used to count how many times certain ISR event occur.
//
//*****************************************************************************
static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
tDMAControlTable sControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(sControlTable, 1024)
tDMAControlTable sControlTable[1024];
#else
tDMAControlTable sControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Declaration of preload structure, referred to by the task list.
// It will be defined later.
//
//*****************************************************************************
tDMAControlTable g_sADC0TaskListPreload;

//*****************************************************************************
//
// This is the task list that defines the DMA scatter-gather operation.
// Each "task" in the task list gets copied one at a time into the alternate
// control structure for the channel, where it is then executed.  Each time
// the DMA channel is triggered, it executes these tasks.
//
//*****************************************************************************
tDMAControlTable g_ADC0TaskTable[] =
{
    //
    // Task 1: transfer ADC data
    // Copy available data from ADC FIFO to buffer
    //
    {
      (void*) (ADC0_BASE + ADC_O_SSFIFO0),              // src addr is ADC0SS0 FIFO
      (void*) &g_ui32ADC0Reading,                       // dst addr ADC read var
      UDMA_CHCTL_SRCSIZE_32 | UDMA_CHCTL_SRCINC_NONE |  // src is a 32 bit word and there is no increment
      UDMA_CHCTL_DSTSIZE_32 | UDMA_CHCTL_DSTINC_NONE |  // dst is a 32 bit word and there is no increment
      UDMA_CHCTL_ARBSIZE_1 |                            // arb size is 1
      ((1 - 1) << UDMA_CHCTL_XFERSIZE_S) |              // transfer size is 1
      UDMA_CHCTL_XFERMODE_MEM_SGA                       // memory scatter-gather
    },

    //
    // Task 2: reset task list
    // Reprogram this DMA channel by reloading the primary structure with
    // the pointers needed to copy the task list for SG mode.
    // Only the control word actually needs to be reloaded.
    // This will allow another DMA transfer to start on this channel the
    // next time a peripheral request occurs.
    //
    {
      &(g_sADC0TaskListPreload.ui32Control),           // src addr is the task reload control word
      &(sControlTable[UDMA_CHANNEL_ADC0].ui32Control), // dst addr is the ADC0 primary control word in the master uDMA control table
      UDMA_CHCTL_SRCSIZE_32 | UDMA_CHCTL_SRCINC_NONE | // src is a 32 bit word and there is no increment
      UDMA_CHCTL_DSTSIZE_32 | UDMA_CHCTL_DSTINC_NONE | // dst is a 32 bit word and there is no increment
      UDMA_CHCTL_ARBSIZE_1 |                           // arb size is 1
      ((1 - 1) << UDMA_CHCTL_XFERSIZE_S) |             // transfer size is 1
      UDMA_CHCTL_XFERMODE_PER_SGA                      // peripheral scatter-gather
    }
};

//*****************************************************************************
//
// This is the preloaded channel control structure for the ADC channel.  The
// values in this structure are configured to perform a memory scatter-gather
// operation whenever a request is received.  This structure is copied over
// into the DMA control table by software at the beginning of this app.  After
// that, it is reloaded into the control table by a scatter-gather operation.
//
// Transfer is 2 uDMA tasks, comprising 4 32-bit words each, for a total of 8 transfers
// Source of the scatter gather transfer is the start of the task list.
// Note that it must point to the last location to copy.
// Destination is the alternate structure for the ADC channel
// Setup transfer parameters for peripheral scatter-gather,
// The arb size is set to 8 to allow the full transfer to happen in one
// transaction.
//
//****************************************************************************
tDMAControlTable g_sADC0TaskListPreload =
{
    &(g_ADC0TaskTable[1].ui32Spare),                                  // src addr is the end of the ADC0 read task list
    &(sControlTable[UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT].ui32Spare),  // dst addr is the end of the ADC0 channel in the master uDMA control table
    UDMA_CHCTL_SRCSIZE_32 | UDMA_CHCTL_SRCINC_32 |                    // src is 32 bit words
    UDMA_CHCTL_DSTSIZE_32 | UDMA_CHCTL_DSTINC_32 |                    // dest is 32 bit words
    UDMA_CHCTL_ARBSIZE_4 |                                            // arb size is 4
    ((8 - 1) << UDMA_CHCTL_XFERSIZE_S) |                              // transfer size is 8
    UDMA_CHCTL_XFERMODE_MEM_SG                                        // memory scatter-gather
};

void
ConfigADC0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 0);
}

//*****************************************************************************
//
// This function loads the channel control structure with the starting values,
// and then enables the DMA channel.
//
//*****************************************************************************
void
ConfigADC0SGTaskList(void)
{
    //
    // Copy the value of the preload task list into the primary structure
    // to initialize it for the first time.  After the DMA starts it will be
    // re-initialized each time by the third s-g task.
    //
    sControlTable[UDMA_CHANNEL_ADC0].pvDstEndAddr = g_sADC0TaskListPreload.pvDstEndAddr;
    sControlTable[UDMA_CHANNEL_ADC0].pvSrcEndAddr = g_sADC0TaskListPreload.pvSrcEndAddr;
    sControlTable[UDMA_CHANNEL_ADC0].ui32Control = g_sADC0TaskListPreload.ui32Control;

    //
    // Enable the ADC DMA channel.  This will allow it to start running when
    // it receives a request from the peripheral
    //
    uDMAChannelEnable(UDMA_CHANNEL_ADC0);
}

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    //
    // Check for uDMA error bit
    //
    uint32_t ui32Status = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status) {
        uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}

int
main(void)
{
    //
    // Set the clocking to run directly from the PLL.  Run at 50 MHz to
    // provide plenty of bus cycles for the DMA operations.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    UARTprintf("\ec\e[?25l");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(sControlTable);
    uDMAIntRegister(INT_UDMAERR, uDMAErrorHandler);
    IntEnable(INT_UDMAERR);

    ConfigADC0();
    ConfigADC0SGTaskList();

    while (true) {
      UARTprintf("\e[H%4u\nErr: %u", g_ui32ADC0Reading, g_ui32uDMAErrCount);
    }
}

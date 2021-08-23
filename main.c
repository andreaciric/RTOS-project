/**
 * @file    main.c
 * @author  Andrea Ciric (ca160202d@student.etf.bg.ac.rs, andreaciric23@gmail.com)
 * @date    2021
 * @brief   ADC conversion starts reading from a sequence of channels (A0, A1) every 1000ms
 * and displays the difference between two value readings from a channel which is chosen by pressing S3 or S4.
 *
 * @Project No.24 -> 011000 => AB==01, C==1, D==0, E==0, F==0
 *
 * @version [1.0 @ 08/2021] Initial version
 */

/* Standard includes. */
#include <ETF5529_HAL/hal_ETF_5529.h>
#include <ETF5529_HAL/hal_7seg.h>

#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Hardware includes. */
#include "msp430.h"

//------------------------------------------------------------------------------------------------------------------------------------------------

#define ULONG_MAX                       0xFFFFFFFF

/** S1_Event bit */
#define mainS3_Event                    ( BIT2 )
/** S2_Event bit */
#define mainS4_Event                    ( BIT3 )

/** Macro for starting the conversion of ADC */
#define adcSTART_CONV                   do {ADC12CTL0 |= ADC12SC;} while (0)

/** Delay used for xTaskTimer to start the conversion */
#define xTaskTimerDelay                 (pdMS_TO_TICKS( 1000 ))

/** "Display task" priority */
#define mainTASK3_PRIO                  ( 1 )
/** "Button task" priority */
#define mainTASK2_PRIO                  ( 2 )
/** task1 priority */
#define mainTASK1_PRIO                  ( 3 )
/* "Timer task" priority */
#define mainTIMER_TASK_PRIO             ( 4 )

#define mainADC_DATA_QUEUE_LEN          ( 16 )

/* This queue will be used to send data from ADC ISR to  Task1 */
QueueHandle_t       xADCQueue = NULL;
/* This queue (mailbox) will be used to send data to display task (Task3) */
QueueHandle_t       xDisplayMailbox = NULL;

/* This handle will be used as Task1 instance*/
TaskHandle_t        xTask1Handler = NULL;
/* This handle will be used as Button task (Task2) instance*/
TaskHandle_t        xButtonTaskHandle = NULL;

/**
 * @brief ADC Channels enum
 */
typedef enum{
    ADC_CHANNEL_0,
    ADC_CHANNEL_1
}adc_channel_t;

/**
 * @brief ADC Data structure
 *
 * This structure encapsulate sData obtained as AD conversion results. Structure sData are
 * written to xADCDataQueue queue from ISR and it is read from "ADC Manager" task
 *
 */
typedef struct{
    adc_channel_t   xChannel;    /**< Mark xChannel from which sData are obtained */
    uint16_t        sData;       /**< Conversion result */
}adc_data_t;

static void prvSetupHardware( void );
//------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief xTaskTimer
 *
 * Task function which starts ADC conversion.
 */
static void xTaskTimer( void *pvParameters )
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for ( ;; )
    {
        adcSTART_CONV;                                          /**< Starting conversion. */
        vTaskDelayUntil( &xLastWakeTime, xTaskTimerDelay );     /**< Delaying the execution of the task. */
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief Task1
 *
 * This task waits for ISR notifications after which it saves last read ADC value and if button is pressed
 * sends value to Display via xDisplayMailbox.
 *
 */
static void xTask1 ( void *pvParameters )
{
    uint32_t ulDataToSend;
    adc_data_t ucLastRead0;
    adc_data_t ucLastRead1;     // results of conversations of both channels
    uint8_t ucSelect = 0;       // if SW3 or SW4 are pressed it hets values 1 or 2
    uint8_t adc_conv, i;
    uint32_t ulBits = 0;        // bits in event group
    adc_data_t xData;

    for(;;)
    {
        /* wait for event */
        xTaskNotifyWait( 0,                 /* don't clear bits on entry */
                         0xffffffff,        /* clear bits on exit */
                         &ulBits,           /* where to store notification value before clear */
                         portMAX_DELAY      /* block until available */
                       );

        /* saves last read ADC conversion */
        adc_conv = (ulBits & (BIT4 | BIT5)) >> 4;
        i = 0;
        while(adc_conv) { i++; adc_conv>>= 1;}
        for (; i > 0;  i--)
        {
            xQueueReceive( xADCQueue, &xData, portMAX_DELAY );
            switch(xData.xChannel)
            {
                case ADC_CHANNEL_0:
                    ucLastRead0 = xData;          //place the result in the variable
                    ulDataToSend = ucLastRead0.sData;
                    if(ucSelect == 1){
                        xQueueOverwrite(xDisplayMailbox, &ulDataToSend);
                        ucSelect = 0;
                    }
                    break;
                case ADC_CHANNEL_1:
                    ucLastRead1 = xData;
                    ulDataToSend = ucLastRead1.sData;
                    if(ucSelect == 2){
                        xQueueOverwrite(xDisplayMailbox, &ulDataToSend);
                        ucSelect = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        /* if buttons are pressed, changes the ucSelect value */
        if( ( ulBits & mainS3_Event ) != 0 ) ucSelect = 1;
        if( ( ulBits & mainS4_Event ) != 0 ) ucSelect = 2;
        ulBits = 0;
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief Task2 - "Button Task" Function
 *
 * This task waits for ISR notification after which it notifies Task1 which button is pressed
 */
static void xTask2( void *pvParameters )
{
    uint16_t i;
    /* Initial button states are 1 because of pull-up configuration */
    uint8_t     currentButtonState  = 1;
    for ( ;; )
    {
        /* waits for notification from ISR*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* wait for a little to check that button is still pressed */
        for(i = 0; i < 1000; i++);
        /* take button state */
        /* check if button SW3 is pressed */
        currentButtonState = ((P1IN & 0x10) >> 4);
        if(currentButtonState == 0){
            /* If SW3 is pressed set bit, defined with mainEVENT_BIT_BUTTON_S3 mask, in
             * Event Group */
            /* set event bit */
            xTaskNotify( xTask1Handler, mainS3_Event, eSetBits );
            continue;
        }
        /* check if button SW4 is pressed*/
        currentButtonState = ((P1IN & 0x20) >> 5);
        if(currentButtonState == 0){
            /* If SW4 is pressed set bit, defined with mainEVENT_BIT_BUTTON_S4 mask, in
             * Event Group */
            /* set event bit */
            xTaskNotify( xTask1Handler, mainS4_Event, eSetBits );
            continue;
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief "Display Task" Function
 *
 * This task read data from xDisplayMailbox but reading is not blocking.
 * xDisplayMailbox is used to send data which will be printed on 7Seg
 * display. After data is received it is decomposed on high and low digit.
 */
static void xTask3( void *pvParameters )
{
    /* New received 8bit data */
    uint8_t         NewValueToShow;
    /* High and Low number digit*/
    uint8_t         digitLow, digitHigh;
    /* Last read value */
    uint16_t        ucLast = 0;
    /* Buffer for receiving values from xTask1 */
    uint16_t        ulBuff;

    for ( ;; )
    {
        /* Check if new number is received*/
        if(xQueueReceive(xDisplayMailbox, &ulBuff, 0) == pdTRUE){
            /* Calculate the difference */
            NewValueToShow = ulBuff - ucLast;
            ucLast = ulBuff;
            /* If there is new number to show on display, split it on High and Low digit */
            /* Extract high digit*/
            digitHigh = NewValueToShow/10;
            /* Extract low digit*/
            digitLow = NewValueToShow - digitHigh*10;
        }
        HAL_7SEG_DISPLAY_1_ON;
        HAL_7SEG_DISPLAY_2_OFF;
        vHAL7SEGWriteDigit(digitLow);
        vTaskDelay(5);
        HAL_7SEG_DISPLAY_2_ON;
        HAL_7SEG_DISPLAY_1_OFF;
        vHAL7SEGWriteDigit(digitHigh);
        vTaskDelay(5);
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief main function
 */
void main( void )
{
    /* Configure peripherals */
    prvSetupHardware();

    /* Create tasks */
    xTaskCreate( xTaskTimer,
                 "TaskTimer",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTIMER_TASK_PRIO,
                 NULL
               );

    xTaskCreate( xTask1,
                 "Task1",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK1_PRIO,
                 &xTask1Handler
               );

    xTaskCreate( xTask2,
                 "Button Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK2_PRIO,
                 &xButtonTaskHandle
               );

    xTaskCreate( xTask3,
                 "Display Task",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK3_PRIO,
                 NULL
               );

    /* Create FreeRTOS objects  */
    xADCQueue           =   xQueueCreate(mainADC_DATA_QUEUE_LEN, sizeof(adc_data_t));
    xDisplayMailbox     =   xQueueCreate(1, sizeof(uint16_t));

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well then this line will never be reached.  If it is reached
    then it is likely that there was insufficient (FreeRTOS) heap memory space
    to create the idle task.  This may have been trapped by the malloc() failed
    hook function, if one is configured. */
    for( ;; );
}

/**
 * @brief Configure hardware upon boot
 */
static void prvSetupHardware( void )
{
    taskDISABLE_INTERRUPTS();

    /* Disable the watchdog. */
    WDTCTL = WDTPW + WDTHOLD;

    hal430SetSystemClock(configCPU_CLOCK_HZ, configLFXT_CLOCK_HZ);

    /* - Init buttons - */
    /*Set direction to input*/
    P1DIR &= ~0x30;
    /*Enable pull-up resistor*/
    P1REN |= 0x30;
    P1OUT |= 0x30;
    /*Enable interrupt for pin connected to SW3*/
    P1IE  |= 0x30;
    P1IFG &=~0x30;
    /*Interrupt is generated during high to low transition*/
    P1IES |= 0x30;

    /*Initialize ADC */
    ADC12CTL0      = ADC12SHT02 + ADC12ON + ADC12MSC;       // Sampling time, ADC12 on
    ADC12CTL1      = ADC12SHP + ADC12CONSEQ_1;              //-  Use sampling timer
                                                            //-  Sampling sequence of channels
    ADC12IE        = 0x02;                       // Enable interrupt for both channels
    ADC12MCTL0     |= ADC12INCH_0;               // ADCMEM0 store result from xChannel 0
    ADC12MCTL1     |= ADC12INCH_1;               // ADCMEM1 store result from xChannel 1
    ADC12MCTL1     |= ADC12EOS;                  // Channel 1 is last xChannel in sequence
    ADC12CTL0      |= ADC12ENC;
    P6SEL          |= 0x03;                      // P6.0 and P6.1 ADC option select

    /* initialize LEDs */
    vHALInitLED();
    /* initialize display*/
    vHAL7SEGInit();
    /*enable global interrupts*/
    taskENABLE_INTERRUPTS();
}

//------------------------------------------------------------------------------------------------------------------------------------------------
void __attribute__ ( ( interrupt( ADC12_VECTOR  ) ) ) vADC12ISR( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    adc_data_t conversionResult;
    switch(__even_in_range(ADC12IV,34))
    {
        case  0: break;                           // Vector  0:  No interrupt
        case  2: break;                           // Vector  2:  ADC overflow
        case  4: break;                           // Vector  4:  ADC timing overflow
        case  6:                                  // Vector  6:  ADC12IFG0
            break;
        case  8:                                  // Vector  8:  ADC12IFG1
            conversionResult.xChannel    = ADC_CHANNEL_0;
            conversionResult.sData       = ADC12MEM0 >> 6;
            xQueueSendToBackFromISR(xADCQueue,&conversionResult,&xHigherPriorityTaskWoken);
            xTaskNotifyFromISR( xTask1Handler, BIT4, eSetBits, &xHigherPriorityTaskWoken );

            conversionResult.xChannel    = ADC_CHANNEL_1;
            conversionResult.sData       = ADC12MEM1 >> 6;
            xQueueSendToBackFromISR(xADCQueue,&conversionResult,&xHigherPriorityTaskWoken);
            xTaskNotifyFromISR( xTask1Handler, BIT5, eSetBits, &xHigherPriorityTaskWoken );
            break;
        case 10: break;                           // Vector 10:  ADC12IFG2
        case 12: break;                           // Vector 12:  ADC12IFG3
        case 14: break;                           // Vector 14:  ADC12IFG4
        case 16: break;                           // Vector 16:  ADC12IFG5
        case 18: break;                           // Vector 18:  ADC12IFG6
        case 20: break;                           // Vector 20:  ADC12IFG7
        case 22: break;                           // Vector 22:  ADC12IFG8
        case 24: break;                           // Vector 24:  ADC12IFG9
        case 26: break;                           // Vector 26:  ADC12IFG10
        case 28: break;                           // Vector 28:  ADC12IFG11
        case 30: break;                           // Vector 30:  ADC12IFG12
        case 32: break;                           // Vector 32:  ADC12IFG13
        case 34: break;                           // Vector 34:  ADC12IFG14
        default: break;
    }
    /* trigger scheduler if higher priority task is woken */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void __attribute__ ( ( interrupt( PORT1_VECTOR  ) ) ) vPORT1ISR( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Give semaphore if button SW3 is pressed*/
    /* Note: This check is not truly necessary but it is good to
     * have it*/
    if(((P1IFG & 0x10) == 0x10)||((P1IFG & 0x20) == 0x20)){
        vTaskNotifyGiveFromISR(xButtonTaskHandle, &xHigherPriorityTaskWoken);
    }
    /*Clear IFG register on exit. Read more about it in offical MSP430F5529 documentation*/
    P1IFG &=~0x30;
    /* trigger scheduler if higher priority task is woken */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* --------------------------------------------------------------------------------------------------------------
 * FreeRTOSBinaryClock.c
 *________________________________________________________________________________
 * A binary clock using FreeRTOS to control the inner workings and timings.
 *________________________________________________________________________________
 * TIME:                            DATE:
 *      Seconds  [ROW 0]     |           Days       [ROW 3]
 *      Minutes  [ROW 1]     |           Months     [ROW 4]
 *      Hours    [ROW 2]     |           Years      [ROW 5]
 *_________________________________________________________
 *STATUS         [ROW 6]                            [ROW 7]
 *_________________________________________________________
 *__________________________________________________________________________________________
 *               MAX VALUES
 *               128    64   32    16     8     4     2     1
 *    Seconds   [   ] [   ] [ X ] [ X ] [ X ] [   ] [ X ] [ X ] 0 -> 59
 *    Minutes   [   ] [   ] [ X ] [ X ] [ X ] [   ] [ X ] [ X ] 0 -> 59
 *    Hours     [   ] [   ] [   ] [ X ] [   ] [ X ] [ X ] [ X ] 0 -> 23
 *    Days      [   ] [   ] [   ] [ X ] [ X ] [ X ] [ X ] [   ] 1 -> 30
 *    Months    [   ] [   ] [   ] [   ] [ X ] [ X ] [   ] [   ] 1 -> 12
 *    Years     [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] 0 -> 255 ([2]000 -> [2]255)
 *    Status    [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ]
 *    Status    [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ] [ X ]
 *__________________________________________________________________________________________
 *________________________________
 * TERMINAL DISPLAY
 *      TIME: hh : mm : ss
 *      DATE: DD / MM / [Y]YYY
 *________________________________
 *___________________________________________________________________________________________________________
 * DESCRIPTION
 * This binary clock uses some features in the FreeRTOS API to function including
 * Binary Semaphores  ,   Queues  , Tasks, Interrupts etc.
 * Along with AVR and C standard library features.
 *
 * TASK DESCRIPTION
 * Task 1: Internal_Clock
 *      Using the Timing of Timer 0 via the semaphore to calculate the timing for a single second.
 *      Which increments every second. Send that data into xQueue1.
 *
 * Task 2: LED_Matrix_Driver
 *      Check xQueue1 for data from Task1 (second data), Use this data to calculate minutes, hours, days
 *      months, years. Send the data to respective rows on the Max7219.
 *
 * Task 3: UART_Driver
 *      Create variables to store current time and date.
 *      Create a separate buffer for each. (You can use one)
 *      Use the itoa() function to convert the uint8_t type def into a char that is represented
 *       as base10 value into the new buffers. Send the buffers as strings to the UART.
 *
 * Task 4: Monitor_Task
 *      Simple loop on row 6 and 7 to simulate a system status display.
 *_________________________________________________________________________________________________________
 * Created: 06/11/2019 14:21:39
 * Author : Graham Claffey
 * -------------------------------------------------------------------------------------------------------- */

/* -------------------------------
 * Set the Arduino clock as 16Mhz.
 * ------------------------------- */
#define F_CPU				16000000UL

/* --------------------------
 * Standard library includes.
 * -------------------------- */
#include <stdio.h>
#include <stdlib.h>

/* ---------------------
 * AVR includes.
 * --------------------- */
#include <avr/interrupt.h>

/* ---------------------
 * FreeRTOS includes.
 * --------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

/* ------------------------------------
 *Define UART BAUD rate and prescaler.
 * ------------------------------------ */
#define BAUD_RATE                9600UL
#define BAUD_PRESCALER           ( ( F_CPU / 16 ) / (BAUD_RATE - 1) )

/* ----------------------
 * On board led testing.
 * ---------------------- */
#define ON_BOARD_LED		PB5
#define BOARD_LED_ON		PORTB |= (1 << ON_BOARD_LED)
#define BOARD_LED_OFF		PORTB &= ~(1 << ON_BOARD_LED)

/* -----------------------------
 * Set output pin definitions.
 * ----------------------------- */
#define DIN				PD7
#define CS				PB0
#define CLK				PB1

/* ------------------------------------------------
 * 8x8 DOT MATRIX CC
 * PIN  13 |  3 |  4 | 10 |  5 | 11 | 15 | 16
 * COL   1 |  2 |  3 |  4 |  5 |  6 |  7 |  8
 *
 * PIN  9 | 14 |  8 | 12 |  1 |  7 |  2 |  5
 * ROW  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8
 * ------------------------------------------------ */
/* -------------------------------
 * Define Max7219 register names.
 * ------------------------------- */
#define NOOP			0  // No-Op : Address 0 = 0x00
#define DIGIT_0			1  // Digit 0 : Address 1 = 0x01
#define DIGIT_1			2  // Digit 1 : Address 2 = 0x02
#define DIGIT_2			3  // Digit 2 : Address 3 = 0x03
#define DIGIT_3			4  // Digit 3 : Address 4 = 0x04
#define DIGIT_4			5  // Digit 4 : Address 5 = 0x05
#define DIGIT_5			6  // Digit 5 : Address 6 = 0x06
#define DIGIT_6			7  // Digit 6 : Address 7 = 0x07
#define DIGIT_7			8  // Digit 7 : Address 8 = 0x08
#define DECODE_MODE		9  // Decode-Mode Register : Address 9 = 0x09 [Data values 0, 1, 15 and 255]
#define INTENSITY		10 // Intensity Register : Address 10 = 0x0A [Data values 0 to 15]
#define SCAN_LIMIT		11 // Scan-Limit Register : Address 11 = 0x0B [Data values 0 to 7]
#define SHUTDOWN		12 // Shutdown Register : Address 12 = 0x0C [Data values 0 and 1]
#define DISPLAY_TEST	15 // Display-Test Register : Address 15 = 0x0F [Data values 0 and 1]

/* ------------------------------------
 *   Task forward declarations.
 * ------------------------------------ */
void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Task3( void *pvParameters );
void Task4( void *pvParameters );

/* ------------------------------------
 *   Task Hook.
 * ------------------------------------ */
void vApplicationIdleHook( void );

/* ----------------------------
 * Port initialization.
 * ---------------------------- */
void vPortsInit( void );

/* ------------------------------------
 *   Timer 0 Initialization.
 * ------------------------------------ */
void vTimer0Init();

/* ------------------------------------
 *   UART forward declarations.
 * ------------------------------------ */
void UARTInit( unsigned char ubrr );
void UARTSendChar( unsigned char charTX );
void UARTSendString( const char *stringTX );

/* ------------------------------------
 *   Max7219 forward declarations.
 * ------------------------------------ */
void vMax7219ClockOut( uint8_t data );
void vMax7219Send( uint8_t regMax, uint8_t data );
void vMax7219Init( void );
void vMax7219Clear( void );

/* -----------------------------------------
 * ISR using Timer 0 output compare vector.
 * ----------------------------------------- */
ISR( TIMER0_COMPA_vect );

/* -----------------------------------------
 * Declaring a Queue handler.
 * ----------------------------------------- */
QueueHandle_t xQueue1 = NULL;

/* -----------------------------------------
 * Declaring a Semaphore Handler.
 * ----------------------------------------- */
xSemaphoreHandle xSemaphore1 = NULL;

/* -----------------------------------------
 * Volatile global variables.
 * ----------------------------------------- */
volatile uint8_t usOverflow;
volatile uint8_t usDigit[8] = {DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4, DIGIT_5, DIGIT_6, DIGIT_7};

/* -----------------------------------------
 * Static global variables.
 * ----------------------------------------- */
static uint8_t usSecond = 0, usMinute = 51, usHour = 21, usDay = 30, usMonth = 11, usYear = 19;

int main(void)
{
    /* -----------------------------------------
     * Initialization setup.
     * ----------------------------------------- */
    cli();
    vPortsInit();
    vTimer0Init();
    UARTInit( BAUD_PRESCALER );
    vMax7219Init();
    sei();

    /* -----------------------------------------
     * Create a Binary Semaphore.
     * ----------------------------------------- */
    xSemaphore1 = xSemaphoreCreateBinary();

    /* -------------------------------------------------------
     * Create a Queue of size 10, with block size of uint8_t.
     * ------------------------------------------------------- */
    xQueue1 = xQueueCreate( 10, sizeof( uint8_t ) );

    /* -----------------------------------------
     * Task creation.
     * ----------------------------------------- */
    xTaskCreate
    (
    Task1
    , ( const portCHAR * ) "Internal_Clock"
    , 128
    , NULL
    , 1
    , NULL
    );

    xTaskCreate
    (
    Task2
    , ( const portCHAR * ) "LED_Matrix_Driver"
    , 128
    , NULL
    , 2
    , NULL
    );

    xTaskCreate
    (
    Task3
    , ( const portCHAR * ) "UART_Driver"
    , 128
    , NULL
    , 3
    , NULL
    );

    xTaskCreate
    (
    Task4
    , ( const portCHAR * ) "Monitor_Task"
    , 128
    , NULL
    , 4
    , NULL
    );

    /* -----------------------------------------
     * Start the Task Scheduler.
     * ----------------------------------------- */
    vTaskStartScheduler();

    while ( 1 )
    {
    }
}

/* -----------------------------------------
 * Port initialization definition.
 * ----------------------------------------- */
void vPortsInit( void )
{
    DDRB |= ( 1 << ON_BOARD_LED | 1 << CS | 1 << CLK );
    DDRD |= ( 1 << DIN );

    PORTB |= ( 1 << CS );
    PORTB &= ~( 1 << ON_BOARD_LED | 1 << CLK );
    PORTD &= ~( 1 << DIN );
}

/* -----------------------------------------
 * Timer 0 Initialization definition.
 * ----------------------------------------- */
void vTimer0Init()
{
    TCCR0A |= ( 1 << WGM01 );       // Set CTC bit
    TCCR0B |= ( 1 << CS02 );        // 256

    OCR0A = 244;                    // Output compare value [244]
    TIMSK0 = ( 1 << OCIE0A );

    usOverflow = 0;
}

/* -----------------------------------------
 * UART Initialization definition.
 * ----------------------------------------- */
void UARTInit( unsigned char ubrr )
{
    UBRR0H = ( unsigned char ) ( ubrr >> 8 );
    UBRR0L = ( unsigned char ) ubrr;

    UCSR0B = ( 1 << RXEN0 ) | ( 1 << TXEN0 );

    UCSR0C = ( 1 << USBS0 ) | ( 3 << UCSZ00 );
}

/* ---------------------------------------------
 * Sending one character at a time to the UART.
 * --------------------------------------------- */
void UARTSendChar( unsigned char charTX )
{
    while( !( UCSR0A & ( 1 << UDRE0 ) ) )
    ;

    UDR0 = charTX;
}

/* -----------------------------------------
 * Send a string to the UART.
 * Uses UARTSendChar until the null
 * termination character is reached.
 * ----------------------------------------- */
void UARTSendString( const char *stringTX )
{
    while( *stringTX != 0x00 )
    {
        UARTSendChar( *stringTX );
        stringTX++;
    }
}

/* -----------------------------------------
 * Make sure CLK is not pulsing.
 * If data is received Bit-shift ones into
 * PORTD, Else 0;
 * Resume CLK.
 * ----------------------------------------- */
void vMax7219ClockOut( uint8_t data )
{
    for( uint8_t i = 0; i < 8; i++ )
    {
        PORTB &= ~( 1 << CLK );

        if( data & ( 1 << ( 7 - i )))
            PORTD |= ( 1 << DIN );
        else PORTD &= ~( 1 << DIN );

        PORTB |= ( 1 << CLK );
    }
}

/* -----------------------------------------
 * Turn off CS, send register and data to
 * vMax7219ClockOut for initialization.
 * Resume CS.
 * ----------------------------------------- */
void vMax7219Send( uint8_t regMax, uint8_t data)
{
    PORTB &= ~( 1 << CS );

    vMax7219ClockOut(regMax);
    vMax7219ClockOut(data);

    PORTB |= ( 1 << CS );
}

/* -----------------------------------------
 * Max7219 Initialization
 * ----------------------------------------- */
void vMax7219Init( void )
{
    vMax7219Send( DECODE_MODE, 0 );
    vMax7219Send( SCAN_LIMIT, 7 );
    vMax7219Send( INTENSITY, 1 );
    vMax7219Clear();
    vMax7219Send( SHUTDOWN, 1 );
}

/* -----------------------------------------
 * Clear the Max7219
 * ----------------------------------------- */
void vMax7219Clear()
{
    for( uint8_t i = 1; i < 9; i++ )
        vMax7219Send( i, 0 );
}

/* -----------------------------------------
 * Task 1: Internal Clock
 * ----------------------------------------- */
void Task1( void *pvParameters )
{
    for( ;; )
    {
        if( xSemaphoreTake( xSemaphore1, portMAX_DELAY) == pdTRUE )
        {
            /* -----------------------------------------------
             * Using the Timing of Timer 0 via the semaphore
             * To calculate the timing for a single second.
             * Which increments every second.
             * Send that data into xQueue1.
             * ----------------------------------------------- */
            usSecond++;
            xQueueSend( xQueue1, &usSecond, portMAX_DELAY );

            if( usSecond > 59 )
            {
                usSecond = 0;
                xQueueSend( xQueue1, ( void * ) &usSecond, ( TickType_t ) portMAX_DELAY );
            }
        }
    }
}

/* -----------------------------------------
 * Task 2: LED Matrix Driver
 * ----------------------------------------- */
void Task2( void *pvParameters )
{
    vMax7219Send( usDigit[1], usMinute);
    vMax7219Send( usDigit[2], usHour);
    vMax7219Send( usDigit[3], usDay);
    vMax7219Send( usDigit[4], usMonth);
    vMax7219Send( usDigit[5], usYear);

    for ( ;; )
    {
        static uint8_t recieve;

        /* ------------------------------------------------
         * Check xQueue1 for data from Task1 (second data)
         * Use this data to calculate minutes, hours, days
         * months, years. Send the data to respective rows
         * on the Max7219.
         * ------------------------------------------------ */
        if( xQueueReceive( xQueue1, &recieve, ( TickType_t ) portMAX_DELAY ) )
        {

            for( int i = 0; i <= recieve; i++ )
            {
                vMax7219Send( usDigit[0], recieve );
                if( recieve == 0 )
                {
                    usMinute++;
                    vMax7219Send( usDigit[1], usMinute);
                }
                if( usMinute > 59 )
                {
                    usMinute = 0;
                    usHour++;
                    vMax7219Send( usDigit[1], usMinute);
                    vMax7219Send( usDigit[2], usHour);
                }
                if( usHour > 23 )
                {
                    usHour = 0;
                    usDay++;
                    vMax7219Send( usDigit[2], usHour);
                    vMax7219Send( usDigit[3], usDay);
                }
                if( usDay > 30 )
                {
                    usDay = 1;
                    usMonth++;
                    vMax7219Send( usDigit[3], usDay);
                    vMax7219Send( usDigit[4], usMonth);
                }
                if( usMonth > 12 )
                {
                    usMonth = 1;
                    usYear++;
                    vMax7219Send( usDigit[4], usMonth);
                    vMax7219Send( usDigit[5], usYear);
                }
            }

        }
    }
}

/* -----------------------------------------
 * Task 3: UART Driver
 * ----------------------------------------- */
void Task3( void *pvParameters )
{
    /* ------------------------------------------------------
     * Create variables to store current time and date.
     * Create a separate buffer for each. (You can use one)
     * Use the itoa() function to convert the uint8_t type def
     * into a char that is represented as base10 value into the
     * new buffers.
     * Send the buffers as strings to the UART.
     * ------------------------------------------------------ */
    for( ;; )
    {
        uint8_t UARTSecond = usSecond;
        uint8_t UARTMinute = usMinute;
        uint8_t UARTHour = usHour;

        uint8_t UARTDay = usDay;
        uint8_t UARTMonth = usMonth;
        uint8_t UARTYear = usYear;

        char bufferTime[10];
        char bufferDate[10];

        UARTSendString("TIME: ");
        itoa( UARTHour, bufferTime, 10 );
        UARTSendString( bufferTime);
        UARTSendString( " : ");

        itoa( UARTMinute, bufferTime, 10 );
        UARTSendString( bufferTime);
        UARTSendString( " : ");

        itoa( UARTSecond, bufferTime, 10 );
        UARTSendString( bufferTime );
        UARTSendChar('\r');
        UARTSendChar('\n');

        UARTSendString("DATE: ");
        itoa( UARTDay, bufferDate, 10 );
        UARTSendString( bufferDate);
        UARTSendString( " / ");

        itoa( UARTMonth, bufferDate, 10 );
        UARTSendString( bufferDate);
        UARTSendString( " / ");

        itoa( UARTYear, bufferDate, 10 );
        UARTSendString("20");
        UARTSendString( bufferDate );

        UARTSendChar('\r');
        UARTSendChar('\n');

        vTaskDelay( 1000 / 1 );

       /* -------------------------------------------
        * Only useful on Tera Term because it
        * uses the: VT100 Terminal Control Commands
        *
        * In this case
        * 27 -> ESC
        * [  -> CSI/Intermediate character
        * 2 and J are Selective parameters
        * of a Parameter string
        -------------------------------------------- */
//         UARTSendChar(27);
//         UARTSendChar('[');
//         UARTSendChar('2');
//         UARTSendChar('J');
    }
}

/* -----------------------------------------
 * Task 4: Monitor Task
 * ----------------------------------------- */
void Task4( void *pvParameters )
{
    /* -----------------------------------------
     * Simple loop on row 6 and 7 to simulate a
     * system status display.
     * ----------------------------------------- */
    for( ;; )
    {
        for( uint8_t i = 0; i < 255; i++ )
        {
            vMax7219Send(usDigit[6], i);
            vMax7219Send(usDigit[7], i);

            vTaskDelay( 1000 / 2 );
        }
    }
}

/* -----------------------------------------
 * vApplicationIdleHook definition
 * ----------------------------------------- */
void vApplicationIdleHook( void )
{
    for( ;; )
    {

    }
}

/* -----------------------------------------------------
 * ISR using Timer 0 output compare vector w/ overflow
 * Since it is an 8-bit timer, the overflow is necessary
 * to have the timer tick at 1 second intervals.
 * ----------------------------------------------------- */
ISR( TIMER0_COMPA_vect )
{
    usOverflow++;

    if( usOverflow >= 255 )
    {
        PORTB ^= ( 1 << ON_BOARD_LED );
        xSemaphoreGiveFromISR( xSemaphore1, NULL );
        usOverflow = 0;
    }
}
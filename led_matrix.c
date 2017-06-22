/*******************************************************************************
 *                                Led Matrix                                   *
 * Version 0.1                                                                 *
 * Author: Benjamin Freeman                                                    *
 *                                                                             *
 * Brief: Led Matrix project using an ESP8266                                  *
 * Features:                                                                   *
 *      - 60 Hz led display                                                    *
 *      - Analog temperature sensor TMP35                                      *
 *      - Motion detection module                                              *
 *      - JSON data fetching with an API                                       *
 *                                                                             *
 *******************************************************************************/
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "esp/spi.h"

/* Wifi settings */
#define WIFI_SSID "MyWifi"
#define WIFI_PASS "MyPass"

/* Matrix settings */

#define MATRIX_ROWS 14

/* Timings for display */

#define T_LOW 0
#define T_HIGH 14
#define T_RESET 15

/* Decade counter pins */
const int ICounterClkPin = 5;
const int IShiftClkPin = 15;
const int ICounterRstPin = 12;

/* Detection pin */
const int IDetectionPin = 4;

/* Display data */
static volatile uint8_t _ucRow = 0;
static volatile uint16_t _pucValues[ MATRIX_ROWS ];

/* Animation - Lines counting up and down */
static volatile bool _pbCountBackward[ MATRIX_ROWS ];

/* Timer count */

static volatile uint8_t _ucFrc1Count = 0;

/* Timer frequency
 *   f = 10 kHz --> p = 100µs */
const uint16_t USFreqFrc1 = 10000;

TickType_t xLastWakeTime;

void vDisplayInterruptHandler( void )
{

    switch ( _ucFrc1Count++ )
    {
        case T_LOW:

            gpio_write( ICounterClkPin, 0 );
            gpio_write( IShiftClkPin, 0 );
            spi_transfer_16( 1, _pucValues[ _ucRow ] );
            gpio_write( IShiftClkPin, 1 );
            _ucRow = ( _ucRow + 1 ) % MATRIX_ROWS;

            break;
        case T_HIGH:

            gpio_write( IShiftClkPin, 0 );
            spi_transfer_16( 1, 0 );

            gpio_write( ICounterClkPin, 1 );
            gpio_write( IShiftClkPin, 1 );

            break;
        case T_RESET:
            _ucFrc1Count = 0;
    }

}

void vFlashAnimation( void )
{
    int i;
    for ( i = 0; i < MATRIX_ROWS; i++ )
    {
        _pucValues[ i ] = ( _pucValues[ i ] == 0 ) ? 16383 : 0;
    }
    vTaskDelay( 200 / portTICK_PERIOD_MS );
}

void vIncrementingValueAnimation( void )
{
    volatile int i;

    for ( i = 0; i < MATRIX_ROWS; i++ )
    {
        _pucValues[ i ] = ( _pucValues[ i ] + 1 ) % 16383;
    }
    vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
}

void vShiftRowAnimation( void )
{
    static int row = 0;
    volatile int i;

    for ( i = 0; i < MATRIX_ROWS; i++ )
    {
        if ( i == row )
        {
            _pucValues[ i ] = i + 1;
        }
        else
        {
            _pucValues[ i ] = 0;
        }
    }
    row = ( row + 1 ) % MATRIX_ROWS;
    vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
}


void vCountingAnimation( void )
{
    volatile int i;

    for ( i = 0; i < MATRIX_ROWS; i++ )
    {
        if ( _pucValues[ i ] == 1 || _pucValues[ i ] == 16383 )
        {
            _pbCountBackward[ i ] = ! _pbCountBackward[ i ];
        }

        _pucValues[ i ] = ( _pbCountBackward[ i ] ) ? _pucValues[ i ] >> 1 : ( _pucValues[ i ] << 1 ) + 1;
    }
    vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
}



void vDisplayTask( void *pvParameters )
{
    int i;
    _pucValues [ 0 ] = 1;
    _pbCountBackward[ 0 ] = true;
    for ( i = 1; i < MATRIX_ROWS; i++ )
    {
        _pucValues[ i ] = ( _pucValues[ i - 1 ] << 1 ) + 1;
        printf("Row number %d : %u\n", i, _pucValues);
        _pbCountBackward[ i ] = false;
    }

   // for ( i = 0; i < MATRIX_ROWS; i++ )
   // {
   //     _pucValues[ i ] = 0;
   // }
    gpio_write( IShiftClkPin, 0 );

    gpio_write( ICounterRstPin, 1 );
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    gpio_write( ICounterRstPin, 0 );

    /* unmask interrupts and start timers */
    timer_set_interrupts( FRC1, true );
    timer_set_run( FRC1, true );



    while( 1 ) {
    //    vFlashAnimation();
        vCountingAnimation();
    }
}

void vTemperatureTask( void *pvParameters )
{
    while( 1 )
    {
        printf( "Temperature is %.1f\n", 330.0 / 1024 * sdk_system_adc_read() );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void vDetectionTask( void *pvParameters )
{
    while( 1 )
    {
        printf( "IDetectionPin %u\n", gpio_read( IDetectionPin ) );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void vPeripheralInit( void )
{
    uart_set_baud( 0, 115200 );

    /* stop both timers and mask their interrupts as a precaution */
    timer_set_interrupts( FRC1, false );
    timer_set_run( FRC1, false );

    /* set up ISRs */
    _xt_isr_attach( INUM_TIMER_FRC1, vDisplayInterruptHandler );

    /* configure timer frequencies */
    timer_set_frequency( FRC1, USFreqFrc1 );

    spi_init( 1, SPI_MODE0, SPI_FREQ_DIV_125K, true, SPI_BIG_ENDIAN, true );

    gpio_enable( ICounterClkPin, GPIO_OUTPUT );
    gpio_enable( IShiftClkPin, GPIO_OUTPUT );
    gpio_enable( ICounterRstPin, GPIO_OUTPUT );

    gpio_enable( IDetectionPin, GPIO_INPUT );

}

void vWifiInit( void )
{
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);
}


void user_init( void )
{

    vPeripheralInit();

    xTaskCreate( vTemperatureTask, "vTemperatureTask", 256, NULL, 1, NULL );
    xTaskCreate( vDetectionTask, "vDetectionTask", 256, NULL, 1, NULL );
    xTaskCreate( vDisplayTask, "vDisplayTask", 256, NULL, 2, NULL );
}

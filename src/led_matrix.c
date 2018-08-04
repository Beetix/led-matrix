/*******************************************************************************
 *                                Led Matrix                                   *
 * Version 0.2                                                                 *
 * Author: Benjamin Freeman                                                    *
 *                                                                             *
 * Brief: Led Matrix project using an ESP8266                                  *
 * Features:                                                                   *
 *      - Binary code modulation to control the Leds' brightness individually  *
 *        (see http://www.batsocks.co.uk/readme/art_bcm_1.htm)                 *
 *      - Analog temperature sensor TMP35                                      *
 *      - Motion detection module                                              *
 *                                                                             *
 *******************************************************************************/
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "esp/spi.h"
#include "ugui.h"

/* Wifi settings */
#define WIFI_SSID "MyWifi"
#define WIFI_PASS "MyPass"

/* Matrix settings */

#define MATRIX_ROWS 14
#define MATRIX_COLUMNS 14

#define TIMESLICES 10
#define EXTRA_TIMER_POW_2_DIV 1
/* Decade counter pins */
const int ICounterClkPin = 5;
const int IShiftClkPin = 15;
const int ICounterRstPin = 12;

/* Detection pin */
const int IDetectionPin = 4;

/* Display data */
static volatile uint16_t _timeSlices[TIMESLICES][ MATRIX_ROWS ];

uint8_t brightness[MATRIX_ROWS][MATRIX_COLUMNS];
static volatile uint8_t _currentSlice = 0;
static volatile uint8_t _currentRow;

TickType_t xLastWakeTime;

UG_GUI gui;

void brightnessesToTimeslices(uint8_t brightness[MATRIX_ROWS][MATRIX_COLUMNS])
{
	for (uint8_t currentSlice = 0; currentSlice < TIMESLICES; currentSlice++)
	{
        for(uint8_t currentRow = 0; currentRow < MATRIX_ROWS; currentRow++)
        {
            uint16_t currentSliceRowBitsValue = 0;
            for ( uint8_t currentColumn = 0; currentColumn < MATRIX_COLUMNS; currentColumn++ )
            {
                if (brightness[currentRow][currentColumn] > currentSlice)
                {
                    currentSliceRowBitsValue |= (1 << currentColumn);
                }
            }
            _timeSlices[currentSlice][currentRow] = currentSliceRowBitsValue;
        }
	}
}

void inline updateDisplay()
{
    brightnessesToTimeslices(brightness);
}

void vDisplayInterruptHandler( void )
{

    timer_set_load(FRC1, 1 << (_currentSlice + EXTRA_TIMER_POW_2_DIV));
    if (_currentSlice == 0)
    {
        if (_currentRow == MATRIX_ROWS - 1)
        {
            gpio_write(ICounterRstPin, 1);
            gpio_write(ICounterRstPin, 0);
        }
        else
        {
            gpio_write(ICounterClkPin, 0);
            gpio_write(ICounterClkPin, 1);
        }

        _currentRow = (_currentRow + 1) % MATRIX_ROWS;
    }

    if (_currentSlice == TIMESLICES)
    {
        gpio_write(IShiftClkPin,0);
        spi_transfer_16(1, 0);
        gpio_write(IShiftClkPin, 1);
    }
    else
    {
        gpio_write(IShiftClkPin,0);
        spi_transfer_16(1, _timeSlices[_currentSlice][_currentRow]);
        gpio_write(IShiftClkPin, 1);
    }

    _currentSlice = (_currentSlice + 1) % (TIMESLICES + 1);
}

void pset(UG_S16 x, UG_S16 y, UG_COLOR col)
{
    brightness[y][x] = col;
}

void leftShiftFrame(int xTopLeft, int yTopLeft, int xBottomRight, int yBottomRight)
{
    for(int col = xTopLeft; col < xBottomRight; col++)
    {
        for(int row = yTopLeft; row < yBottomRight; row++)
        {
            brightness[row][col] = brightness[row][col + 1];
        }
    }
}

void scrollText(char text[], const UG_FONT* font )
{
    char * currentChar = text;
    while(*currentChar)
    {
        for(int x = 0; x < font->char_width; x++)
        {
            leftShiftFrame(0, 0, MATRIX_COLUMNS - 1, font->char_height);
            int index = ( *currentChar - font->start_char)* font->char_height;
            for(int y = 0; y < font->char_height; y++)
            {
                if (font->p[index++] & (1 << x))
                    pset(14-1,y,2);
                else
                    pset(14-1,y,0);
            }

            updateDisplay();
            vTaskDelayUntil( &xLastWakeTime, 200 / portTICK_PERIOD_MS );
        }
        currentChar++;
    }
    for(int col=0; col < MATRIX_COLUMNS; col++)
    {
        leftShiftFrame(0, 0, MATRIX_COLUMNS - 1, font->char_height);

        for(int y = 0; y < font->char_height; y++)
        {
            pset(14-1,y,0);
        }

        updateDisplay();
        vTaskDelayUntil( &xLastWakeTime, 200 / portTICK_PERIOD_MS );
    }
}

void vDisplayTask( void *pvParameters )
{
    UG_COLOR foreBrightness = 2;
    UG_Init(&gui, pset, 14, 14);
    UG_SetForecolor(foreBrightness);
    UG_SetBackcolor(0);

    /* unmask interrupts and start timers */
    timer_set_interrupts( FRC1, true );
    timer_set_run( FRC1, true );

    UG_FontSelect(&FONT_8X12);

    char greeting[] = "LED MATRIX";
    scrollText(greeting, &FONT_8X12);

    UG_FontSelect(&FONT_4X6);
    char displayChar = 'A';

    while (1)
    {
        char c = uart_getc_nowait(0);
        if (c == 'u')
        {
            printf("Increasing brightness to %d\n", foreBrightness + 1);
            foreBrightness++;
            UG_SetForecolor(foreBrightness);
        }
        else if (c == 'd')
        {
            printf("Decreasing brightness to %d\n", foreBrightness - 1);
            foreBrightness--;
            UG_SetForecolor(foreBrightness);
        }
        else if (c == 'r')
        {
            printf("Reseting brightness\n");
            UG_FillScreen(0);
            updateDisplay();
        }
        else if (c == 'a')
        {
            printf("Arrow\n");
            brightness[0][0] = foreBrightness;
            brightness[1][0] = foreBrightness;
            brightness[1][1] = foreBrightness;
            brightness[2][0] = foreBrightness;
            updateDisplay();
        }
        else if (c == 'c')
        {
            printf("Calibration\n");
            brightness[0][0] = foreBrightness;
            brightness[1][1] = foreBrightness;
            brightness[MATRIX_ROWS-1][MATRIX_COLUMNS-1] = foreBrightness;
            updateDisplay();
        }
        else if (c == 'g')
        {
            printf("Drawing rectangle\n");
            UG_FillFrame(0,0,4,4,foreBrightness);
            updateDisplay();
        }
        else if (c == 't')
        {
            printf("Drawing text\n");
            UG_PutString(0,0,"42");
            updateDisplay();
        }
        else if (c == 'y')
        {
            printf("Char demo\n");
            UG_FontSelect(&FONT_8X12);
            do
            {
                UG_PutChar(displayChar++,0,0,foreBrightness,0);
                updateDisplay();
                vTaskDelayUntil( &xLastWakeTime, 500 / portTICK_PERIOD_MS );
            } while (uart_getc_nowait(0) != 'y');
            UG_FontSelect(&FONT_4X6);
        }
        else if (c == 'b')
        {
            printf("Brightness demo\n");
            do
            {
                UG_FillFrame(5,5,10,10,foreBrightness);
                updateDisplay();
                foreBrightness = (foreBrightness + 1) % TIMESLICES;
                vTaskDelayUntil( &xLastWakeTime, 500 / portTICK_PERIOD_MS );
            } while (uart_getc_nowait(0) != 'b');
        }
        else if (c == 'w')
        {
            printf("Temperature demo\n");
            do
            {
                int temp = 330.0 / 1024 * sdk_system_adc_read();
                UG_PutChar(temp / 10 + '0', 0, 5, foreBrightness, 0);
                UG_PutChar(temp % 10 + '0', 4, 5, foreBrightness, 0);
                UG_PutChar('c', 9, 5, foreBrightness, 0);

                pset(8,5,foreBrightness);
                pset(10,5,foreBrightness);
                pset(9,4,foreBrightness);
                pset(9,6,foreBrightness);

                updateDisplay();
                vTaskDelayUntil( &xLastWakeTime, 500 / portTICK_PERIOD_MS );
            } while (uart_getc_nowait(0) != 'w');
        }
        else if (c == 'p')
        {
            printf("Presence demo\n");
            UG_FontSelect(&FONT_8X12);
            bool toggle = false;
            do
            {
                if (gpio_read( IDetectionPin ))
                {
                    if (toggle)
                    {
                        UG_PutChar(3, 3, 2, foreBrightness, 0);
                    }
                    else
                    {
                        UG_PutChar(1, 3, 2, foreBrightness, 0);
                    }
                    toggle = !toggle;
                }
                else
                {
                    UG_PutChar(0, 3, 2, foreBrightness, 0);
                }
                updateDisplay();
                vTaskDelayUntil( &xLastWakeTime, 1000 / portTICK_PERIOD_MS );
            } while (uart_getc_nowait(0) != 'p');
            UG_FontSelect(&FONT_4X6);
        }


        vTaskDelayUntil( &xLastWakeTime, 100 / portTICK_PERIOD_MS );
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
    timer_set_divider(FRC1, TIMER_CLKDIV_16);
    timer_set_load(FRC1, (1 << EXTRA_TIMER_POW_2_DIV));
    timer_set_reload(FRC1, false);

    spi_init( 1, SPI_MODE0, SPI_FREQ_DIV_1M, true, SPI_BIG_ENDIAN, true );

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

    sdk_wifi_set_opmode(NULL_MODE);
    vPeripheralInit();

//    xTaskCreate( vTemperatureTask, "vTemperatureTask", 256, NULL, 1, NULL );
//    xTaskCreate( vDetectionTask, "vDetectionTask", 256, NULL, 1, NULL );
    xTaskCreate( vDisplayTask, "vDisplayTask", 256, NULL, 2, NULL );
}

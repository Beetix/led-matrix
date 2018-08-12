#include "FreeRTOS.h"
#include "task.h"
#include "temperature.h"

void vTemperatureTask( void *pvParameters )
{
    while( 1 )
    {
        printf( "Temperature is %.1f\n", 330.0 / 1024 * sdk_system_adc_read() );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

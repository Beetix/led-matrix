#include "FreeRTOS.h"
#include "task.h"
#include "espressif/esp_common.h"
#include "esp8266.h"
#include "detection.h"

/* Detection pin */
const int IDetectionPin = 4;

void detectionInit()
{
    gpio_enable( IDetectionPin, GPIO_INPUT );
}

void vDetectionTask( void *pvParameters )
{
    while( 1 )
    {
        printf( "IDetectionPin %u\n", detection() );
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

bool detection()
{
    return gpio_read(IDetectionPin);
}

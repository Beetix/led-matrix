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
#include "display.h"
#include "detection.h"
#include "temperature.h"

/* Wifi settings */
#define WIFI_SSID "MyWifi"
#define WIFI_PASS "MyPass"

void vPeripheralInit( void )
{
    uart_set_baud( 0, 115200 );

    displayInit();
    detectionInit();
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

/*
 *  Examples read temperature, pressure, humidity from BME280
 *
 *  BME280 connected to
 *  I2C SDA - GPIO2
 *  I2C SCL - GPIO0
 *
 */

#include "ets_sys.h"
#include "driver/i2c.h"
#include "driver/i2c_bme280.h"
#include "driver/uart.h"
#include "osapi.h"
#include "os_type.h"
#include "user_interface.h"
#include "user_config.h"

os_event_t user_procTaskQueue[user_procTaskQueueLen];
extern int ets_uart_printf(const char *fmt, ...);
int (*console_printf)(const char *fmt, ...) = ets_uart_printf;
static void user_procTask(os_event_t *events);
static volatile os_timer_t sensor_timer;

void sensor_timerfunc(void *arg)
{

    BME280_readSensorData();

    signed long int temp;
    temp = BME280_GetTemperature();
    unsigned long int press;
    press = BME280_GetPressure();
    unsigned long int hum;
    hum = BME280_GetHumidity();

    ets_uart_printf("Temp: %d.%d DegC, ", (int)(temp/100), (int)(temp%100));
    ets_uart_printf("Pres: %d.%d hPa, ", (int)(press/100), (int)(press%100));
    ets_uart_printf("Hum: %d.%d pct \r\n", (int)(hum/1024), (int)(hum%1024));

}

static void ICACHE_FLASH_ATTR
user_procTask(os_event_t *events)
{
    os_delay_us(5000);
}

void user_rf_pre_init(void)
{

}

void user_init(void)
{
    // Init uart
    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    os_delay_us(1000);

    ets_uart_printf("\r\nBooting...\r\n");

    // Init
    if (BME280_Init(BME280_MODE_FORCED) ) {
        //Disarm timer
        os_timer_disarm(&sensor_timer);
        //Setup timer
        os_timer_setfn(&sensor_timer, (os_timer_func_t *)sensor_timerfunc, NULL);
        //Arm timer for every 5 sec.
        os_timer_arm(&sensor_timer, 5000, 1);
    }else{
        ets_uart_printf("BME280 init error.\r\n");
    }

    system_os_task(user_procTask, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);

}

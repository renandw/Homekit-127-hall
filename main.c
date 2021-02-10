#include <stdio.h>
#include <stdlib.h>
#include <espressif/esp_wifi.h>
#include "espressif/esp_common.h"
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include "i2c/i2c.h"
#include <bh1750/bh1750.h>

#define  led_gpio 2 // The GPIO pin that is oconnected to built-in LED on ESP12F or D4 on wemos D1 mini.

//LUX SENSOR PINS
#define SCL_PIN 5 // Wemos D1
#define SDA_PIN 4 // Wemos D2
#define I2C_BUS 0
#define SENSOR_POLL_PERIOD 2000  // reading time

#define FIRMWARE_VERSION "1.0.0"
#define MANUFACTURER_DEFAULT "HomeKid™"
#define MODEL_DEFAULT " LightSensor"


//OCCUPANCY SENSORS PINS
//Sensor PIN
#define SENSOR_PIN 14
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif
//Sensor PIN 2
#define SENSOR_PIN_2 16
#ifndef SENSOR_PIN_2
#error SENSOR_PIN_2 is not specified
#endif


//HOMEKIT CHARACTERISTIC SECTION
homekit_characteristic_t occupancy_detected = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t occupancy_detected_2 = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t lux = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0, .min_step = (float[]) {0.01}, .min_value = (float[]) {0}, .max_value = (float[]) {100000}); // 
homekit_characteristic_t fault = HOMEKIT_CHARACTERISTIC_(STATUS_FAULT, 0);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  MANUFACTURER_DEFAULT);
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL, MODEL_DEFAULT);
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  FIRMWARE_VERSION);
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, NULL);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, NULL);


void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}
void gpio_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(false);
}

uint16_t bh1750_reading(i2c_dev_t *dev);



//Reset Configuration
void reset_configuration_task() {
    //Flash the LED first before we start the reset
    for (int i = 0; i<5; i++) {
            gpio_write(led_gpio, false);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            gpio_write(led_gpio, true);
            vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    printf("Resetting Wifi Config\n");    
    wifi_config_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Resetting HomeKit Config\n");
    homekit_server_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Restarting\n");
    sdk_system_restart();
    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL); 
}

//Identification
void identify_task(void *_args) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            gpio_write(led_gpio, false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_write(led_gpio, true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


//SENSOR IDENTIFY
void sensor_identify(homekit_value_t _value) {
    printf("LightSensor identify\n");
    xTaskCreate(identify_task, "Identify", 128, NULL, 2, NULL);
}
void occupancy_identify(homekit_value_t _value) {
    printf("Occupancy identify\n");
}

//LUX SENSOR SECTION
void sensor_task(void *_args) {
   float lux_value;
      i2c_dev_t dev = {
      .addr = BH1750_ADDR_LO,
      .bus = I2C_BUS,
    };
    bh1750_configure(&dev, BH1750_CONTINUOUS_MODE | BH1750_HIGH_RES_MODE);  //BH1750_HIGH_RES_MODE = 1lux, BH1750_HIGH_RES_MODE2  = 0.5lux resolution
    while (1) {
        lux_value = (float)bh1750_read(&dev);
        printf("Lux: %.1f\n", lux_value);
        lux.value = HOMEKIT_FLOAT(lux_value);
        homekit_characteristic_notify(&lux, lux.value);
        vTaskDelay(SENSOR_POLL_PERIOD / portTICK_PERIOD_MS);
        }
    }

void sensor_init() {
    xTaskCreate(sensor_task, "Sensor Task", 256, NULL, 2, NULL);
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
}

//OCCUPANCY SENSOR SECTION

void sensor_callback(bool high, void *context) {
    occupancy_detected.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&occupancy_detected, occupancy_detected.value);
}

void sensor_callback_2(bool high, void *context) {
    occupancy_detected_2.value = HOMEKIT_UINT8(high ? 1 : 0);
    homekit_characteristic_notify(&occupancy_detected_2, occupancy_detected_2.value);
}



//HOMEKIT ACCESSORIES SECTION
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, sensor_identify),
            NULL
        }),
            HOMEKIT_SERVICE(LIGHT_SENSOR, .primary=true,  .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Ambient Light Sensor"),
            &lux,
            &fault,
            NULL
        }),
        NULL
}),
    HOMEKIT_ACCESSORY(.id=2, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            &serial,
            HOMEKIT_CHARACTERISTIC(MODEL, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
  }),
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
            &occupancy_detected,
            NULL
        }),
        NULL
  }),
    HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "1"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Occupancy Sensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
}),      
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .primary=false, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            &occupancy_detected_2,
            NULL
      }),
      NULL
  }),
  NULL
};


homekit_server_config_t config = {
    .accessories = accessories,
    .password = "736-24-212",
    .setupId="73NA" /// homekit_accessory_category_sensor = 10,
};

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);

    int name_len = snprintf(NULL, 0, "LightSensor-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "LightSensor-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);

    name.value = HOMEKIT_STRING(name_value);
    
    char *serial_value = malloc(13);
    snprintf(serial_value, 13, "%02X%02X%02X%02X%02X%02X", macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
    serial.value = HOMEKIT_STRING(serial_value);


}

void on_wifi_ready() {
    homekit_server_init(&config);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    create_accessory_name();
    gpio_init();
    sensor_init();
    wifi_config_init("LuxMotion", "12345678", on_wifi_ready);

    if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
    printf("Failed to initialize sensor\n");
    }
    if (toggle_create(SENSOR_PIN_2, sensor_callback_2, NULL)) {
    printf("Failed to initialize sensor\n");
    }
}

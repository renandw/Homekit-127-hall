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
#include <button.h>
#include <toggle.h>
#include "ota-api.h"

//LUX SENSOR PINS
#define SCL_PIN 5 // Wemos D1
#define SDA_PIN 4 // Wemos D2
#define I2C_BUS 0
#define SENSOR_POLL_PERIOD 2000  // reading time

#define ALLOWED_FACTORY_RESET_TIME 30000

//OCCUPANCY SENSORS PINS
//Sensor PIN
#define SENSOR_PIN 13
#ifndef SENSOR_PIN
#error SENSOR_PIN is not specified
#endif
//Sensor PIN 2
#define SENSOR_PIN_2 16
#ifndef SENSOR_PIN_2
#error SENSOR_PIN_2 is not specified
#endif

//RELAYS PINS
// The GPIO pin that is connected to RELAY#1 on the board.
const int relay_gpio_1 = 0;
// The GPIO pin that is connected to RELAY#2 on the board.
const int relay_gpio_2 = 2;

//TOGGLE PINS
#define TOGGLE_PIN_1 12
#ifndef TOGGLE_PIN_1
#error TOGGLE_PIN_1 is not specified
#endif

#define TOGGLE_PIN_2 14
#ifndef TOGGLE_PIN_2
#error TOGGLE_PIN_2 is not specified
#endif


//HOMEKIT CHARACTERISTIC SECTION
homekit_characteristic_t occupancy_detected = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t occupancy_detected_2 = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t lux = HOMEKIT_CHARACTERISTIC_(CURRENT_AMBIENT_LIGHT_LEVEL, 0, .min_step = (float[]) {0.01}, .min_value = (float[]) {0}, .max_value = (float[]) {100000}); // 
homekit_characteristic_t fault = HOMEKIT_CHARACTERISTIC_(STATUS_FAULT, 0);
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");
homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "LuxMotionRelay");



uint16_t bh1750_reading(i2c_dev_t *dev);



//Reset Configuration
void reset_configuration_task() {
    printf("Resetting Wifi due to toggle pin under ALLOWED_FACTORY_RESET_TIME");
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
    if (xTaskGetTickCountFromISR() < ALLOWED_FACTORY_RESET_TIME / portTICK_PERIOD_MS) {
    xTaskCreate(reset_configuration_task, "Reset configuration", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
} else {
    printf("Factory reset not allowed after %ims since boot. Repower device and try again\n", ALLOWED_FACTORY_RESET_TIME);
}
}


//ACCESSORIES IDENTIFY
void sensor_identify(homekit_value_t _value) {
    printf("LightSensor identify\n");
}
void occupancy_identify(homekit_value_t _value) {
    printf("Occupancy identify\n");
}
void light_identify(homekit_value_t _value) {
    printf("Light identify\n");
}

//RELAY CONFIGURATION

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);
void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);


void relay_write_1(bool on) {
    gpio_write(relay_gpio_1, on ? 0 : 1);
}

void relay_write_2(bool on) {
    gpio_write(relay_gpio_2, on ? 0 : 1);
}

homekit_characteristic_t lightbulb_on_1 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_1_callback)
);

homekit_characteristic_t lightbulb_on_2 = HOMEKIT_CHARACTERISTIC_(
    ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(lightbulb_on_2_callback)
);


void gpio_init() {
  
    gpio_enable(relay_gpio_1, GPIO_OUTPUT);
    relay_write_1(lightbulb_on_1.value.bool_value);

    gpio_enable(relay_gpio_2, GPIO_OUTPUT);
    relay_write_2(lightbulb_on_2.value.bool_value);

    gpio_enable(TOGGLE_PIN_1, GPIO_INPUT);
    gpio_enable(TOGGLE_PIN_2, GPIO_INPUT);
}

void lightbulb_on_1_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_1(lightbulb_on_1.value.bool_value);
}

void lightbulb_on_2_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write_2(lightbulb_on_2.value.bool_value);
}

//TOGGLE CALLBACKS

void toggle_callback_1(bool high, void *context) {
    printf("toggle is %s\n", high ? "high" : "low");
    lightbulb_on_1.value.bool_value = !lightbulb_on_1.value.bool_value;
    relay_write_1(lightbulb_on_1.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_1, lightbulb_on_1.value);
}

void toggle_callback_2(bool high, void *context) {
    printf("toggle is %s\n", high ? "high" : "low");
    lightbulb_on_2.value.bool_value = !lightbulb_on_2.value.bool_value;
    relay_write_2(lightbulb_on_2.value.bool_value);
    homekit_characteristic_notify(&lightbulb_on_2, lightbulb_on_2.value);
    reset_configuration();
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
      HOMEKIT_ACCESSORY(
          .id=1,
          .category=homekit_accessory_category_switch,
          .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
          HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
          &name,
          &manufacturer,
          &serial,
          &model,
          &revision,
        NULL
    }),

    HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
       HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 01"),
        &lightbulb_on_1,
        &ota_trigger,
        NULL
    }),
    NULL,
  }),
  HOMEKIT_ACCESSORY(
        .id=2,
        .category=homekit_accessory_category_switch,
        .services=(homekit_service_t*[]){
          HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(IDENTIFY, light_identify),
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 02"),
            &manufacturer,
            &serial,
            &model,
            &revision,
            NULL
      }),  

      HOMEKIT_SERVICE(LIGHTBULB, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Lâmpada 02"), 
            &lightbulb_on_2,
            NULL
      }),
      NULL
  }),
    HOMEKIT_ACCESSORY(.id=3, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
            NULL
}),      
        HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor_2"),
            &occupancy_detected_2,
            NULL
      }),
      NULL
  }),
  HOMEKIT_ACCESSORY(.id=4, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
      HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
          HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
          &manufacturer,
          &serial,
          &model,
          &revision,
          HOMEKIT_CHARACTERISTIC(IDENTIFY, occupancy_identify),
          NULL
}),
      HOMEKIT_SERVICE(OCCUPANCY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
          HOMEKIT_CHARACTERISTIC(NAME, "Occupancy Sensor"),
          &occupancy_detected,
          NULL
      }),
      NULL
}),

      HOMEKIT_ACCESSORY(.id=5, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
          HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
              HOMEKIT_CHARACTERISTIC(NAME, "Ambient Light Sensor"),
              &manufacturer,
              &serial,
              &model,
              &revision,
              HOMEKIT_CHARACTERISTIC(IDENTIFY, sensor_identify),
              NULL
          }),
              HOMEKIT_SERVICE(LIGHT_SENSOR,  .characteristics=(homekit_characteristic_t*[]) {
              HOMEKIT_CHARACTERISTIC(NAME, "Ambient Light Sensor"),
              &lux,
              &fault,
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
}


void user_init(void) {
    uart_set_baud(0, 115200);
    create_accessory_name();
    gpio_init();
    sensor_init();

    if (toggle_create(SENSOR_PIN, sensor_callback, NULL)) {
    printf("Failed to initialize sensor\n");
    }
    if (toggle_create(SENSOR_PIN_2, sensor_callback_2, NULL)) {
    printf("Failed to initialize sensor\n");
    }
    if (toggle_create(TOGGLE_PIN_1, toggle_callback_1, NULL)) {
    printf("Failed to initialize toggle 1 \n");
    }
    if (toggle_create(TOGGLE_PIN_2, toggle_callback_2, NULL)) {
    printf("Failed to initialize toggle 2 \n");
    }

    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;

    homekit_server_init(&config);
}

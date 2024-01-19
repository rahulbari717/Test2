/*****************************************************************************
   Q) 1. Write a program that runs on an ESP32 development board (using ESP-IDF). This program will:
  
    a. Log data from an MPU9050 6 axis IMU chip at precisely 500Hz (along with a
       timestamp)
    b. Send this data in real time via MQTT to an AWS instance

   Modify the above program such that the LED blinks at 1 Hz via OTA updates hosted
          on AWS 
 
 
 * ***************************************************************************
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *

 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "driver/i2c.h"

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"
#include "mqtt_client.h"

static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

/*
  
    Replace "mqtt://your-aws-broker-url" with your actual AWS broker URL. This code initializes 
    the MQTT client, runs an IMU task, and publishes data to the specified MQTT topic. You'll need to 
    implement the logic to fetch and format the IMU data into the publish_mqtt_data

*/
#define MQTT_BROKER_URL "mqtt://your-aws-broker-url" // "mqtt://your-aws-broker-url" with your actual AWS broker URL
#define MQTT_TOPIC "esp32/imu"

#define OTA_UPDATE_TOPIC "esp32/ota/update" // MQTT topic for OTA updates

#define OTA_URL "http://your-ota-server/ota_update.bin"
#define LED_PIN GPIO_NUM_2


calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    .mag_scale = {.x = 1.513449, .y = 1.557811, .z = 1.434039},
    .accel_offset = {.x = 0.020900, .y = 0.014688, .z = -0.002580},
    .accel_scale_lo = {.x = -0.992052, .y = -0.990010, .z = -1.011147},
    .accel_scale_hi = {.x = 1.013558, .y = 1.011903, .z = 1.019645},

    .gyro_bias_offset = {.x = 0.303956, .y = -1.049768, .z = -0.403782}};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void)
{

  i2c_mpu9250_init(&cal);
  ahrs_init(SAMPLE_FREQ_Hz, 0.8);

  uint64_t i = 0;
  while (true)
  {
    vector_t va, vg, vm;

    // Get the Accelerometer, Gyroscope and Magnetometer values.
    ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

    // Transform these values to the orientation of our device.
    transform_accel_gyro(&va);
    transform_accel_gyro(&vg);
    transform_mag(&vm);

    // Apply the AHRS algorithm
    ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                va.x, va.y, va.z,
                vm.x, vm.y, vm.z);

    // Print the data out every 10 items
    if (i++ % 10 == 0)
    {
      float temp;
      ESP_ERROR_CHECK(get_temperature_celsius(&temp));

      float heading, pitch, roll;
      ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
      ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);

      // Make the WDT happy
      vTaskDelay(0);
    }

    pause();
  }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}



static esp_mqtt_client_handle_t mqtt_client;
calibration_t cal = {
    .mag_offset = {.x = 25.183594, .y = 57.519531, .z = -62.648438},
    // ... (other calibration values)
};



static esp_spp_cb_t spp_callback;

// Function to handle OTA updates
static void perform_ota_update()
{
    esp_http_client_config_t config = {.url = OTA_URL};
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        ESP_LOGI("OTA", "OTA Update Successful");
        esp_restart();
    }
    else
    {
        ESP_LOGE("OTA", "OTA Update Failed");
    }

    esp_http_client_cleanup(client);
}

// OTA update MQTT event handler
static void ota_update_event_handler(esp_mqtt_event_handle_t event)
{
    if (event->event_id == MQTT_EVENT_DATA)
    {
        // Check if the received MQTT message is for OTA update
        if (strncmp(event->data, "OTA_UPDATE", event->data_len) == 0)
        {
            ESP_LOGI("OTA", "Received OTA update request");
            perform_ota_update();
        }
    }
}

// OTA update task
void ota_update_task(void *params)
{
    SemaphoreHandle_t ota_semaphore = (SemaphoreHandle_t)params;

    while (true)
    {
        // Wait for the semaphore to trigger OTA update
        if (xSemaphoreTake(ota_semaphore, portMAX_DELAY))
        {
            perform_ota_update();
        }
    }
}

// Function to update LED state and send it over Bluetooth
static void update_and_send_led_state(bool led_state)
{
    // Update LED state
    gpio_set_level(LED_PIN, led_state);

    // Send LED state over Bluetooth
    char message[20];
    snprintf(message, sizeof(message), "LED State: %s", led_state ? "ON" : "OFF");
    esp_spp_write(param->write.handle, strlen(message), (uint8_t *)message);
}

// Function to publish data via MQTT
static void publish_mqtt_data()
{
    char msg[100];
    // Fetch and format IMU data into msg
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, msg, 0, 1, 0);
}

// MQTT event handler
static void mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("mqtt", "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("mqtt", "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("mqtt", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("mqtt", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("mqtt", "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("mqtt", "MQTT_EVENT_ERROR");
        break;
    }
}


void app_main(void)
{
  // Initialize MQTT
  esp_mqtt_client_config_t mqtt_cfg = {.uri = MQTT_BROKER_URL, .event_handle = mqtt_event_handler};
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_start(mqtt_client);

  // start imu task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);

  // Initialize Bluetooth Serial (SPP)
  esp_spp_init(&spp_callback);

  

}
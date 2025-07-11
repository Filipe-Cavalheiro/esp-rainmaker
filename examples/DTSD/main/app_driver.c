/* Includes ------------------------------------------------------------------*/
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include "app_priv.h"

#define DEFAULT_SATURATION  100
#define DEFAULT_BRIGHTNESS  50

static const char *TAG_DRIVER = "app_driver";

float g_temperature;
float g_humidity;
uint16_t g_lux;
bool g_motion;

#include <driver/gpio.h>
#include <esp_err.h>

// DHT22 Sensor Task
#define DHT22_TASK_STACK_SIZE 4096
#define DHT22_TASK_PRIORITY 4
#define DHT22_TASK_CORE_ID 1

#define DHT_GPIO GPIO_NUM_5 // GPIO pin connected to the DHT22

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

//ADC OF LDR
//#define LDR_GPIO GPIO_NUM_34 // GPIO pin connected to the LDR
#include "/home/fil2k/esp/esp-idf/components/esp_adc/include/esp_adc/adc_oneshot.h"
static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t LDR_ADC_CHANNEL = ADC_CHANNEL_6;

//ThingsSpeak
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");
#include "esp_http_client.h"

//Motion sensor
#define PIR_GPIO GPIO_NUM_4  // GPIO for PIR Sensor

/**
 * Starts DHT22 sensor task
 * @return DHT_OK if successful, otherwise error code
 */
void DHT22_task_start(void);

/**
 * Stops DHT22 sensor task
 * @return DHT_OK if successful, otherwise error code
 */
void errorHandler(int response);

/**
 * Get the signal level
 * @param usTimeOut Timeout
 * @param state State of the signal
 * @return uSec is number of microseconds passed
 */
int getSignalLevel(int usTimeOut, bool state);

/**
 * @brief Read integer data from sensor on specified pin
 *
 * Humidity and temperature are returned as integers.
 * For example: humidity=625 is 62.5 %, temperature=244 is 24.4 degrees Celsius
 *
 * @param sensor_type DHT11 or DHT22
 * @param pin GPIO pin connected to sensor OUT
 * @param[out] humidity Humidity, percents * 10, nullable
 * @param[out] temperature Temperature, degrees Celsius * 10, nullable
 * @return `ESP_OK` on success
 */
esp_err_t dht_read_data();

float app_get_current_temperature();

////////////////////////////// DHT22 CODE //////////////////////////////

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

// DHT timer precision in microseconds
#define DHT_TIMER_INTERVAL 2
#define DHT_DATA_BITS 40
#define DHT_DATA_BYTES (DHT_DATA_BITS / 8)

// == global defines =============================================

static const char *TAG_DHTX = "DHTX";

float humidity = 0.;
float temperature = 0.;

#define PORT_ENTER_CRITICAL() portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL() portEXIT_CRITICAL(&mux)


// == error handler ===============================================

/**
 * Error handler
 * @param response
 */
void errorHandler(int response)
{
    switch (response)
    {

    case DHT_TIMEOUT_ERROR:
        printf("Sensor Timeout\n");
        break;

    case DHT_CHECKSUM_ERROR:
        printf("CheckSum error\n");
        break;

    case DHT_OK:
        break;

    default:
        printf("Unknown error\n");
    }
}

/**
 * Get the signal level
 * @param usTimeOut Timeout
 * @param state State of the signal
 * @return uSec is number of microseconds passed
 */

int getSignalLevel(int usTimeOut, bool state)
{
    int uSec = 0;
    while (gpio_get_level(DHT_GPIO) == state)
    {
        if (uSec++ > usTimeOut)
        {
            return -1; // Timeout
        }
        esp_rom_delay_us(1);
    }
    return uSec;
}

///////////////////////////////////////////////


/**
 * Pack two data bytes into single value and take into account sign bit.
 */

static inline int16_t dht_convert_data(uint8_t msb, uint8_t lsb)
{
    int16_t data;
    data = msb & 0x7F;
    data <<= 8;
    data |= lsb;

    if (msb & BIT(7))
        data = -data; // convert it to negative

    return data;
}


static inline esp_err_t dht_fetch_data(uint8_t data[DHT_DATA_BYTES])
{
    int uSec = 0;
    uint8_t byteInx = 0;
    uint8_t bitInx = 7;

    for (int k = 0; k < DHT_DATA_BYTES; k++)
        data[k] = 0;

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    esp_rom_delay_us(3000);

    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(25);

    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    // Step through Phase 'B', 40us
    uSec = getSignalLevel(85, 0);
    if (uSec < 0)
        return ESP_FAIL;

    uSec = getSignalLevel(85, 1);
    if (uSec < 0)
        return ESP_FAIL;

    for (int k = 0; k < DHT_DATA_BITS; k++)
    {
        uSec = getSignalLevel(56, 0);
        if (uSec < 0)
            return ESP_FAIL;

        uSec = getSignalLevel(75, 1);
        if (uSec < 0)
            return ESP_FAIL;

        if (uSec > 40)
        {
            data[byteInx] |= (1 << bitInx);
        }

        if (bitInx == 0)
        {
            bitInx = 7;
            ++byteInx;
        }
        else
            bitInx--;
    }

    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
        return ESP_OK;
    else
        return ESP_FAIL;
}

esp_err_t dht_read_data()
{
    uint8_t data[DHT_DATA_BYTES] = {0};

    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DHT_GPIO, 1);

    esp_err_t result = dht_fetch_data(data);

    if (result == ESP_OK)
    {
        // PORT_EXIT_CRITICAL();
    }

    /* restore GPIO direction because, after calling dht_fetch_data(), the
     * GPIO direction mode changes */
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT_OD);

    gpio_set_level(DHT_GPIO, 1);

    if (result != ESP_OK)
    {
        printf("==== dht_read_data - ESP_NOT_OK ====\n");
        return result;
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
    {
        ESP_LOGE(TAG_DHTX, "Checksum failed, invalid data received from sensor");

        return ESP_ERR_INVALID_CRC;
    }

    humidity = dht_convert_data(data[0], data[1]) / 10;
    temperature = dht_convert_data(data[2], data[3]) / 10;

    ESP_LOGI(TAG_DHTX, "Sensor data: humidity=%f, temp=%f", humidity, temperature);

    return ESP_OK;
}

////////////////////////////// END OF DHT22 CODE //////////////////////////////

////////////////////////////// START OF GPIO CODE //////////////////////////////

esp_err_t gpio_read_data()
{
    int raw_value = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw_value));

    g_lux = raw_value;  

    ESP_LOGI(TAG_DRIVER, "LDR ADC Raw Value: %d", raw_value);
    return ESP_OK;
}

////////////////////////////// END OF GPIO CODE //////////////////////////////

/**
 * DHT22 Sensor task - Reads temperature and humidity, then updates ESP-Rainmaker parameters
 */
static void DHT22_task(void *pvParameter)
{
    ESP_LOGI(TAG_DRIVER, "Starting DHT task\n\n");

    for (;;)
    {
        ESP_LOGI(TAG_DRIVER, "=== Reading DHT ===\n");

        // Update ESP-Rainmaker parameters with new sensor data
        dht_read_data();
        g_temperature = temperature; 
        g_humidity = humidity;        

        ESP_LOGI(TAG_DRIVER, "Updated Temperature: %f C, Humidity: %f %%", g_temperature, g_humidity);

        // Report temperature
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(g_temperature)
        );

        // Report humidity
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(humidity_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(g_humidity)
        );

        // Wait before reading again (30 seconds or defined interval)
        vTaskDelay(DEFAULT_MEASURE_INTERVAL / portTICK_PERIOD_MS);
    }
}

/**
 * LDR Sensor task - Reads light level, then updates ESP-Rainmaker parameters
 */
static void LDR_task(void *pvParameter)
{
    ESP_LOGI(TAG_DRIVER, "Starting LDR task\n\n");

    for (;;)
    {
        ESP_LOGI(TAG_DRIVER, "=== Reading LDR ===\n");

        // Update ESP-Rainmaker parameters with new sensor data
        gpio_read_data();   

        ESP_LOGI(TAG_DRIVER, "Updated LDR: %d lux %%", g_lux);

        // Report the updated temperature and humidity to ESP-Rainmaker
        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(lux_sensor_device, ESP_RMAKER_PARAM_BRIGHTNESS),
            esp_rmaker_int(g_lux)
        );

        // Wait before reading again (30 seconds or defined interval)
        vTaskDelay(DEFAULT_MEASURE_INTERVAL / portTICK_PERIOD_MS);
    }
}

static void PIR_task(void *pvParameter)
{
    ESP_LOGI(TAG_DRIVER, "Starting PIR sensor task\n\n");

    // Configure GPIO for input mode
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    for (;;)
    {
        ESP_LOGI(TAG_DRIVER, "=== Reading PIR ===\n");
        g_motion = gpio_get_level(PIR_GPIO);

        esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(pir_sensor_device, ESP_RMAKER_PARAM_POWER),
            esp_rmaker_bool(g_motion)
        );

        // Wait before reading again (adjustable interval, e.g., every second)
        vTaskDelay(DEFAULT_MEASURE_INTERVAL / portTICK_PERIOD_MS);
    }
}

esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

void thingspeak_task(void *pvParameters)
{
    char post_data[1000];
    esp_err_t err;
    
    esp_http_client_config_t config_post = {
        .url = "https://api.thingspeak.com/update?",
        .method = HTTP_METHOD_POST,
        .cert_pem = (char *)server_cert_pem_start,
        .event_handler = client_event_post_handler,
        .transport_type = HTTP_TRANSPORT_OVER_SSL
    };

    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    
    /*
    esp_http_client_config_t config_post = {
        .url = "http://httpbin.org/post",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    */

    for (;;)
    {
        vTaskDelay(DEFAULT_MEASURE_INTERVAL / portTICK_PERIOD_MS);

        snprintf(post_data, sizeof(post_data),
                 "api_key=%s&field1=%f&field2=%f&field3=%d&field4=%d",
                 THINGSPEAK_WRITE_API_KEY, g_humidity, g_temperature, g_lux, g_motion);

        ESP_LOGI(TAG_DRIVER, "Request URL: %s", post_data);

        esp_http_client_handle_t client = esp_http_client_init(&config_post);
        esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");


        esp_http_client_set_post_field(client, post_data, strlen(post_data));
        
        err = esp_http_client_perform(client);

        if (err == ESP_OK)
        {
            int status_code = esp_http_client_get_status_code(client);
            if (status_code == 200)
            {
                ESP_LOGI(TAG_DRIVER, "Message sent Successfully");
            }
            else
            {
                ESP_LOGI(TAG_DRIVER, "Unexpected HTTP status: %d", status_code);
            }
        }
        else
        {
            ESP_LOGI(TAG_DRIVER, "HTTP request failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);
    }

    vTaskDelete(NULL);
}


float app_get_current_temperature()
{
    return g_temperature;
}

float app_get_current_humidity()
{
    return g_humidity;
}

float app_get_current_lux()
{
    return g_lux;
}

bool app_get_current_motion()
{
    return g_motion;
}

/**
 * Initialize temperature and humidity reading task
 */
esp_err_t app_temperature_and_humidity_init(void)
{
    // Start DHT22 sensor task
    xTaskCreatePinnedToCore(&DHT22_task, "DHT22_task", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

/**
 * Initialize light reading task
 */
esp_err_t app_light_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,  // good for wider voltage range up to ~3.6V
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL, &chan_cfg));

    // Start DHT22 sensor task
    xTaskCreatePinnedToCore(&LDR_task, "LDR_task", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

/**
 * Initialize thingspeak task
 */
esp_err_t thingspeak_init(void){
    // Start thingspeak sensor task
    xTaskCreatePinnedToCore(&thingspeak_task, "thingspeak_task", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}

/**
 * Initialize PIR sensor task
 */
esp_err_t app_pir_init(void)
{
    xTaskCreatePinnedToCore(&PIR_task, "PIR_task", 4096, NULL, 5, NULL, 0);
    return ESP_OK;
}


/**
 * Driver initialization function
 */
void app_driver_init_sensors()
{
    ESP_LOGI(TAG_DRIVER, "Initializing all taskes...");
    app_temperature_and_humidity_init();  // Initialize the temperature and humidity sensor task
    app_light_init();
    thingspeak_init();
    app_pir_init();
}

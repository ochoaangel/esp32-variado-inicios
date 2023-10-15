/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//----------------- Mido la distancia con un AJ-SR04M -----------------//
/////////////////////////////////////////////////////////////////////////
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#define ECHO_PIN GPIO_NUM_23
#define TRIGGER_PIN GPIO_NUM_22

// #define MAX_DISTANCE_CM 400

volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile bool measurement_done = false;

void IRAM_ATTR echo_isr_handler(void *arg)
{
  if (gpio_get_level(ECHO_PIN) == 1)
  {
    start_time = esp_timer_get_time();
  }
  else
  {
    end_time = esp_timer_get_time();
    measurement_done = true;
  }
}

void setup()
{
  gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

  gpio_set_intr_type(ECHO_PIN, GPIO_INTR_ANYEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, NULL);
  // gpio_install_isr_service(0);
}

void measure_distance_task(void *pvParameters)
{
  while (1)
  {
    gpio_set_level(TRIGGER_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    // vTaskDelay(2 / portTICK_PERIOD_MS);

    gpio_set_level(TRIGGER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    // vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN, 0);

    while (!measurement_done)
    {
      vTaskDelay(pdMS_TO_TICKS(1));
      // vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    uint32_t pulse_duration = end_time - start_time;
    float distance = pulse_duration * 0.034 / 2;

    printf("Tiempo de inicio: %lu, Tiempo final: %lu, Distancia: %.2f cm\n", start_time, end_time, distance);

    measurement_done = false;
    vTaskDelay(pdMS_TO_TICKS(5000));
    // vTaskDelay(5000 / portTICK_PERIOD_MS); // Espera 5 segundos antes de la siguiente medida
  }
}

void app_main()
{
  setup();

  xTaskCreatePinnedToCore(measure_distance_task, "measure_distance_task", 2048, NULL, 1, NULL, 0);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//------------------------- Se conecta al wifi -------------------------//
/////////////////////////////////////////////////////////////////////////

// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "nvs_flash.h"
// #include "lwip/err.h"
// #include "lwip/sys.h"
// #include "esp_netif.h"
// #include "esp_event.h" // Include esp_event.h instead
// #include "sntp.h"
// #include "time.h"

// #define WIFI_SSID "SHAW-EEAD50"
// #define WIFI_PASSWORD "0MGSM253KPJ9"

// static const char *TAG = "wifi_example";

// static void obtain_time(void) {
//     ESP_LOGI(TAG, "Obteniendo la hora actual...");
//     time_t now;
//     struct tm timeinfo;
//     time(&now);
//     localtime_r(&now, &timeinfo);
//     char strftime_buf[64];
//     strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//     ESP_LOGI(TAG, "Hora actual: %s", strftime_buf);
// }

// static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         ESP_LOGI(TAG, "------No se pudo conectar al WiFi");
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ESP_LOGI(TAG, "-------Conectado al WiFi");
//         obtain_time();
//     }
// }

// void wifi_init_sta(void) {
//     esp_netif_init();
//     esp_event_loop_create_default();
//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);
//     esp_event_handler_instance_t instance_any_id;
//     esp_event_handler_instance_t instance_got_ip;
//     esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
//     esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);
//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = WIFI_SSID,
//             .password = WIFI_PASSWORD,
//             .scan_method = WIFI_FAST_SCAN,
//             .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
//             .threshold.rssi = -127,
//             .threshold.authmode = WIFI_AUTH_WPA2_PSK,
//         },
//     };
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
//     esp_wifi_start();
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "------- iniciando tudu ---------------");
//     ESP_ERROR_CHECK(nvs_flash_init());
//     wifi_init_sta();
// }

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//-------------------- contador de 0-60 repetitivo --------------------//
/////////////////////////////////////////////////////////////////////////
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// TaskHandle_t taskHandle;

// void taskFunction(void *parameter) {
//   int counter = 0;
//   while (1) {
//     printf("_____%d\n", counter);
//     counter = (counter + 1) % 60;
//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }

// void app_main(void) {
//   xTaskCreatePinnedToCore(taskFunction, "Task", 2048, NULL, 1, &taskHandle, 0);
// }
///////////////////////////////////////////////////
///////////////////////////////////////////////////
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/Task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "driver/gpio.h"
// #include "driver/uart.h"
// #include "esp_task_wdt.h"

// static const int RX_BUF_SIZE = 1024;

// #define TXD_PIN (GPIO_NUM_17)
// #define RXD_PIN (GPIO_NUM_16)

// #define UART UART_NUM_2

// int num = 0;

// void init(void)
// {
//     const uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };

//     // We won't use a buffer for sending data.
//     uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART, &uart_config);
//     uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// static void tx_task(void *arg)
// {
// 	char* Txdata = (char*) malloc(100);
//     while (1) {
//     	sprintf (Txdata, "Hello world index = %d\r\n", num++);
//         uart_write_bytes(UART, Txdata, strlen(Txdata));
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     while (1) {
//         // const int rxBytes = uart_read_bytes(UART, data, RX_BUF_SIZE, 500 / portTICK_RATE_MS);
//         const int rxBytes = uart_read_bytes(UART, data, RX_BUF_SIZE, pdMS_TO_TICKS(100));
//         if (rxBytes > 0) {
//             data[rxBytes] = 0;
//             // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
//             printf("Read %d bytes: '%s'", rxBytes, data);
//         }
//     }
//     free(data);
// }

// void app_main(void)
// {
//     init();
//     xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
//     xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
// }

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/Task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "driver/gpio.h"
// #include "driver/uart.h"
// #include "esp_task_wdt.h"

// // static const char *tag = "UARTEVENT";
// #define TXD_PIN (GPIO_NUM_17)
// #define RXD_PIN (GPIO_NUM_16)
// #define UART UART_NUM_2
// #define BUF_SIZE 1024
// #define TASK_MEMORY 1024 * 2

// static QueueHandle_t uart_queue;

// static void uart_task(void *pvParameters)
// {
//     uart_event_t event;
//     uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
//     while (1)
//     {
//         if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
//         {
//             bzero(data, BUF_SIZE);
//             switch (event.type)
//             {
//             case UART_DATA:
//                 printf("Testtttttt");
//                 uart_read_bytes(UART, data, event.size, pdMS_TO_TICKS(100));
//                 uart_write_bytes(UART, (const char *)data, event.size);
//                 uart_flush(UART);
//                 break;

//             default:
//                 break;
//             }
//         }
//     }
// }

// static void init_uart(void)
// {
//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         // .source_clk = UART_SCLK_APB,
//     };
//     uart_param_config(UART, &uart_config);
//     uart_set_pin(UART,  TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART, BUF_SIZE, BUF_SIZE, BUF_SIZE, &uart_queue, 0);
//     xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, configMAX_PRIORITIES-1, NULL);
// }

// void app_main()
// {
//     init_uart();
// }

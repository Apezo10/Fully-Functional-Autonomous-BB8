#include "ESP_WiFi_initialization.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"


//Name of the WiFi network that esp connects to
static constexpr char WIFI_SSID[] = "Pezo Family";
static constexpr char WIFI_PASSWORD[] = "Benandadin";

static const char* WIFI_TAG = "BB8_WIFI";

//FreeRTOS event group used to track connection
static EventGroupHandle_t wifiEventGroup;
static TaskHandle_t wifiReconnectTaskHandle = nullptr;

//Bit that becomes 1 after eps gets IP
static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr uint32_t WIFI_RECONNECT_DELAY_MS = 1000;

static void wifiReconnectTask(void*) {
    while (true) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        vTaskDelay(
            pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS)
        );

        esp_wifi_connect();
    }
}

static void requestWiFiConnect() {
    if (wifiReconnectTaskHandle != nullptr) {
        xTaskNotifyGive(wifiReconnectTaskHandle);
    }
}


//This function is called whenever WiFi or IP event occur
static void wifiEventHandler(
    void*,
    esp_event_base_t eventBase,
    int32_t eventId,
    void* eventData
) {
    if (
        eventBase == WIFI_EVENT &&
        eventId == WIFI_EVENT_STA_START
    ) {
        requestWiFiConnect();
    }

    else if (
        eventBase == WIFI_EVENT &&
        eventId == WIFI_EVENT_STA_DISCONNECTED
    ) {
        ESP_LOGW(WIFI_TAG, "Disconnected. Reconnecting...");

        xEventGroupClearBits(
            wifiEventGroup,
            WIFI_CONNECTED_BIT
        );

        requestWiFiConnect();
    }

    else if (
        eventBase == IP_EVENT &&
        eventId == IP_EVENT_STA_GOT_IP
    ) {
        auto* event =
            static_cast<ip_event_got_ip_t*>(eventData);

        ESP_LOGI(
            WIFI_TAG,
            "ESP32 IP address: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );

        xEventGroupSetBits(
            wifiEventGroup,
            WIFI_CONNECTED_BIT
        );
    }
}

//Create function called in header
bool connectWiFi() {
    wifiEventGroup = xEventGroupCreate();

    if (wifiReconnectTaskHandle == nullptr) {
        BaseType_t taskResult = xTaskCreatePinnedToCore(
            wifiReconnectTask,
            "wifi_reconnect",
            3072,
            nullptr,
            4,
            &wifiReconnectTaskHandle,
            1
        );

        if (taskResult != pdPASS) {
            ESP_LOGE(
                WIFI_TAG,
                "Failed to create reconnect task"
            );

            return false;
        }
    }

    esp_err_t result = nvs_flash_init();

    if (
        result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(result);
    }

    ESP_ERROR_CHECK(esp_netif_init());

    result = esp_event_loop_create_default();

    // It may already exist in a larger project.
    if (
        result != ESP_OK &&
        result != ESP_ERR_INVALID_STATE
    ) {
        ESP_ERROR_CHECK(result);
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifiInitConfig =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&wifiInitConfig)
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifiEventHandler,
            nullptr
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifiEventHandler,
            nullptr
        )
    );

    wifi_config_t wifiConfig = {};

    std::strncpy(
        reinterpret_cast<char*>(wifiConfig.sta.ssid),
        WIFI_SSID,
        sizeof(wifiConfig.sta.ssid) - 1
    );

    std::strncpy(
        reinterpret_cast<char*>(wifiConfig.sta.password),
        WIFI_PASSWORD,
        sizeof(wifiConfig.sta.password) - 1
    );

    wifiConfig.sta.pmf_cfg.capable = true;
    wifiConfig.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifiConfig
        )
    );

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(
        WIFI_TAG,
        "Connecting to %s...",
        WIFI_SSID
    );

    EventBits_t bits = xEventGroupWaitBits(
        wifiEventGroup,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(15000)
    );

    return (bits & WIFI_CONNECTED_BIT) != 0;
}

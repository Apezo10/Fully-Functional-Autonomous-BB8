#include "ESP_UDP_reciever.h"

#include <cerrno>
#include <cstdio>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

static constexpr int UDP_PORT = 4210;
static const char* UDP_TAG = "BB8_UDP";

// Latest command received from the Raspberry Pi.
static float latestForward = 0.0f;
static float latestTurn = 0.0f;
static int64_t lastCommandTimeUs = 0;

// Protects the command variables because one task writes them
// while the main loop reads them.
static portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t udpTaskHandle = nullptr;


// Keeps a value between a minimum and maximum.
static float clampFloat(
    float value,
    float minimum,
    float maximum
) {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}


// Runs continuously as a FreeRTOS background task.
static void udpReceiverTask(void*) {
    // Create an IPv4 UDP socket.
    int socketFile = socket(
        AF_INET,
        SOCK_DGRAM,
        IPPROTO_IP
    );

    if (socketFile < 0) {
        ESP_LOGE(
            UDP_TAG,
            "Socket creation failed. errno=%d",
            errno
        );

        udpTaskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    // Listen on every ESP32 network interface using port 4210.
    sockaddr_in serverAddress = {};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(UDP_PORT);
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);

    if (
        bind(
            socketFile,
            reinterpret_cast<sockaddr*>(&serverAddress),
            sizeof(serverAddress)
        ) < 0
    ) {
        ESP_LOGE(
            UDP_TAG,
            "Socket bind failed. errno=%d",
            errno
        );

        close(socketFile);
        udpTaskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(
        UDP_TAG,
        "Listening on UDP port %d",
        UDP_PORT
    );

    char receiveBuffer[64];

    while (true) {
        sockaddr_in senderAddress = {};
        socklen_t senderAddressLength =
            sizeof(senderAddress);

        // Wait here until a packet arrives.
        int receivedLength = recvfrom(
            socketFile,
            receiveBuffer,
            sizeof(receiveBuffer) - 1,
            0,
            reinterpret_cast<sockaddr*>(&senderAddress),
            &senderAddressLength
        );

        if (receivedLength < 0) {
            ESP_LOGE(
                UDP_TAG,
                "recvfrom failed. errno=%d",
                errno
            );

            continue;
        }

        // Convert received bytes into a C string.
        receiveBuffer[receivedLength] = '\0';

        float receivedForward = 0.0f;
        float receivedTurn = 0.0f;

        // Expected format:
        // forward,turn
        //
        // Example:
        // 0.25,-0.10
        int parsedValues = std::sscanf(
            receiveBuffer,
            "%f,%f",
            &receivedForward,
            &receivedTurn
        );

        if (parsedValues != 2) {
            ESP_LOGW(
                UDP_TAG,
                "Invalid packet: %s",
                receiveBuffer
            );

            continue;
        }

        receivedForward = clampFloat(
            receivedForward,
            -1.0f,
            1.0f
        );

        receivedTurn = clampFloat(
            receivedTurn,
            -1.0f,
            1.0f
        );

        // Safely update the shared command variables.
        portENTER_CRITICAL(&commandMux);

        latestForward = receivedForward;
        latestTurn = receivedTurn;
        lastCommandTimeUs = esp_timer_get_time();

        portEXIT_CRITICAL(&commandMux);

        ESP_LOGI(
            UDP_TAG,
            "Forward: %.2f | Turn: %.2f",
            receivedForward,
            receivedTurn
        );
    }
}


bool startUDPReceiver() {
    // Avoid accidentally creating the task twice.
    if (udpTaskHandle != nullptr) {
        return true;
    }

    BaseType_t result = xTaskCreate(
        udpReceiverTask,
        "udp_receiver",
        4096,
        nullptr,
        5,
        &udpTaskHandle
    );

    if (result != pdPASS) {
        udpTaskHandle = nullptr;

        ESP_LOGE(
            UDP_TAG,
            "Failed to create UDP task"
        );

        return false;
    }

    return true;
}


bool getLatestPiCommand(
    float& forward,
    float& turn,
    uint32_t& commandAgeMs
) {
    int64_t copiedCommandTimeUs = 0;

    portENTER_CRITICAL(&commandMux);

    forward = latestForward;
    turn = latestTurn;
    copiedCommandTimeUs = lastCommandTimeUs;

    portEXIT_CRITICAL(&commandMux);

    // No packet has been received yet.
    if (copiedCommandTimeUs == 0) {
        commandAgeMs = UINT32_MAX;
        return false;
    }

    int64_t currentTimeUs = esp_timer_get_time();

    commandAgeMs = static_cast<uint32_t>(
        (currentTimeUs - copiedCommandTimeUs) / 1000
    );

    return true;
}
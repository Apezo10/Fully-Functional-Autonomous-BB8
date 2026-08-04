set(srcs
    "main.c"
    "sketch.cpp"
    "ESP_WiFi_initialization.cpp"
    "ESP_UDP_reciever.cpp"
)

set(requires
    "bluepad32"
    "bluepad32_arduino"
    "arduino"
    "btstack"
    "esp_wifi"
    "esp_event"
    "esp_netif"
    "nvs_flash"
    "lwip"
    "esp_timer"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
)
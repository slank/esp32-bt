idf_component_register(
    SRCS
        "main.c"
        "hci_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        freertos
        log
        esp_event
)

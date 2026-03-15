#pragma once

#include "esp_err.h"

/**
 * Initialise the HCI bridge.
 *
 * Must be called after the BT controller has been initialised and enabled in
 * BLE-only mode, and after TinyUSB CDC-ACM has been set up.
 *
 * Registers VHCI callbacks and starts the internal send/receive tasks.
 */
esp_err_t hci_bridge_init(void);

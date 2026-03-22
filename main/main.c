/* SPDX-License-Identifier: Apache-2.0 */

/*
 * app_main — initialise NVS, BT controller (BLE-only), TinyUSB CDC-ACM,
 * then start the HCI H:4 bridge.
 *
 * Linux bring-up:
 *   sudo btattach -B /dev/ttyACM0 -P h4 -S 115200
 *   sudo btmgmt info
 *   bluetoothctl
 */

#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "hci_bridge.h"

static const char *TAG = "main";

/* Declared in hci_bridge.c – registered as the CDC RX callback below. */
extern void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event);

void app_main(void)
{
    /* ── NVS (BT uses it for PHY calibration data) ──────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── BT controller (BLE only, VHCI mode) ───────────────────────────── */
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means classic BT memory was already released
         * (e.g. by sdkconfig), which is fine. */
        ESP_LOGW(TAG, "mem_release(CLASSIC_BT): %s", esp_err_to_name(ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_LOGI(TAG, "BT controller started in BLE mode");

    /* ── TinyUSB CDC-ACM ────────────────────────────────────────────────── */
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port              = TINYUSB_CDC_ACM_0,
        .callback_rx           = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    ESP_LOGI(TAG, "USB CDC-ACM ready");

    /* ── HCI bridge ─────────────────────────────────────────────────────── */
    ESP_ERROR_CHECK(hci_bridge_init());
    ESP_LOGI(TAG, "Ready — attach with: sudo btattach -B /dev/ttyACM0 -P h4 -S 115200");
}

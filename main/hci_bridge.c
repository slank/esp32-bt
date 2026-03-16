/*
 * HCI H:4 bridge — ESP32-S3 VHCI ↔ TinyUSB CDC-ACM
 *
 * Data flow:
 *   Linux ──USB CDC──► CDC RX callback ──s_to_vhci_q──► vhci_send_task ──► BT controller
 *   BT controller ──► vhci_notify_recv ──s_from_vhci_q──► usb_send_task ──► USB CDC TX
 *
 * Both queues are necessary:
 *   - to-VHCI:   flow-controlled by the VHCI "send available" semaphore.
 *   - from-VHCI: decouples the BT-task callback from TinyUSB writes, which
 *                must happen from a regular FreeRTOS task (not a callback) so
 *                that write_flush can block until the USB endpoint is free.
 *
 * The VHCI interface delivers/receives complete HCI packets that already carry
 * the H:4 packet-indicator byte as their first byte (0x01=CMD, 0x02=ACL,
 * 0x04=EVT).  Both directions are therefore simple pass-through once a
 * complete packet has been reassembled from the CDC RX byte stream.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "tinyusb_cdc_acm.h"
#include "hci_bridge.h"

static const char *TAG = "hci_bridge";

/* ── Constants ──────────────────────────────────────────────────────────── */

/* Maximum size of a single HCI packet we will buffer.
 * HCI CMD/EVT params ≤ 255 bytes + up to 5-byte header = 260.
 * HCI ACL data for BLE ≤ 251 bytes payload + 4-byte header + 1 type = 256.
 * Use 512 for comfortable headroom. */
#define MAX_HCI_PKT  512

/* Number of packet slots in each direction queue. */
#define QUEUE_DEPTH  8

/* H:4 packet-type indicators */
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04

/* ── Types ──────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t  data[MAX_HCI_PKT];
    uint16_t len;
} hci_pkt_t;

/* H:4 receive state machine (for reassembling host→controller packets). */
typedef enum {
    RX_WAIT_TYPE,    /* waiting for the 1-byte H:4 type indicator */
    RX_WAIT_HDR,     /* accumulating fixed header bytes for this packet type */
    RX_WAIT_PAYLOAD, /* accumulating variable-length payload */
} h4_rx_state_t;

/* ── Module state ───────────────────────────────────────────────────────── */

/* Queue of complete packets waiting to be forwarded to the VHCI (host→ctrl). */
static QueueHandle_t     s_to_vhci_q;

/* Queue of complete packets received from the VHCI, waiting to be sent to the
 * USB host (ctrl→host).  Populated in the BT-task callback; drained by
 * usb_send_task so that TinyUSB writes happen from a proper task context. */
static QueueHandle_t     s_from_vhci_q;

/* Semaphore released by notify_host_send_available: signals the VHCI send
 * task that the controller is ready to accept another packet. */
static SemaphoreHandle_t s_send_avail_sem;

/* H:4 reassembly state (single-threaded: only touched by CDC RX callback). */
static h4_rx_state_t s_rx_state = RX_WAIT_TYPE;
static hci_pkt_t     s_rx_pkt;          /* packet being assembled */
static uint16_t      s_rx_pos;          /* bytes written into s_rx_pkt.data so far */
static uint16_t      s_rx_hdr_need;     /* header bytes still needed */
static uint16_t      s_rx_payload_need; /* payload bytes still needed */

/* ── H:4 header-length table ────────────────────────────────────────────── */

/* Returns the number of bytes that follow the type byte before we can
 * determine the payload length.  Returns 0 for unknown types. */
static inline uint8_t h4_hdr_len(uint8_t type)
{
    switch (type) {
    case H4_CMD: return 3; /* opcode(2) + param_len(1) */
    case H4_ACL: return 4; /* handle(2) + total_len(2) */
    case H4_SCO: return 3; /* handle(2) + data_len(1)  */
    case H4_EVT: return 2; /* event_code(1) + param_len(1) */
    default:     return 0;
    }
}

/* Returns the payload length encoded in the header of a partially assembled
 * packet.  Called once we have (1 + hdr_len) bytes. */
static uint16_t h4_payload_len(const uint8_t *pkt)
{
    switch (pkt[0]) {
    case H4_CMD: return (uint16_t)pkt[3];
    case H4_ACL: return (uint16_t)(pkt[3] | (pkt[4] << 8));
    case H4_SCO: return (uint16_t)pkt[3];
    case H4_EVT: return (uint16_t)pkt[2];
    default:     return 0;
    }
}

/* ── H:4 reassembly ─────────────────────────────────────────────────────── */

/* Feed one byte into the reassembly state machine.
 * Calls enqueue_to_vhci() when a complete packet is ready. */
static void h4_rx_byte(uint8_t b)
{
    switch (s_rx_state) {

    case RX_WAIT_TYPE:
        s_rx_pkt.data[0] = b;
        s_rx_pos = 1;
        s_rx_hdr_need = h4_hdr_len(b);
        if (s_rx_hdr_need == 0) {
            /* Unknown type – skip silently and stay in WAIT_TYPE. */
            ESP_LOGW(TAG, "Unknown H:4 type 0x%02x, skipping", b);
            break;
        }
        s_rx_state = RX_WAIT_HDR;
        break;

    case RX_WAIT_HDR:
        if (s_rx_pos < MAX_HCI_PKT) {
            s_rx_pkt.data[s_rx_pos++] = b;
        }
        if (--s_rx_hdr_need == 0) {
            s_rx_payload_need = h4_payload_len(s_rx_pkt.data);
            if (s_rx_payload_need == 0) {
                /* Zero-length payload: packet is complete. */
                s_rx_pkt.len = s_rx_pos;
                hci_pkt_t copy = s_rx_pkt;
                xQueueSend(s_to_vhci_q, &copy, 0);
                s_rx_state = RX_WAIT_TYPE;
            } else {
                s_rx_state = RX_WAIT_PAYLOAD;
            }
        }
        break;

    case RX_WAIT_PAYLOAD:
        if (s_rx_pos < MAX_HCI_PKT) {
            s_rx_pkt.data[s_rx_pos++] = b;
        }
        if (--s_rx_payload_need == 0) {
            s_rx_pkt.len = s_rx_pos;
            hci_pkt_t copy = s_rx_pkt;
            if (xQueueSend(s_to_vhci_q, &copy, 0) != pdTRUE) {
                ESP_LOGW(TAG, "to-VHCI queue full, packet dropped");
            }
            s_rx_state = RX_WAIT_TYPE;
        }
        break;
    }
}

/* ── USB CDC callbacks ──────────────────────────────────────────────────── */

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    (void)itf;
    (void)event;

    static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t rx_size = 0;

    esp_err_t ret = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf,
                                        CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC read error: %s", esp_err_to_name(ret));
        return;
    }

    for (size_t i = 0; i < rx_size; i++) {
        h4_rx_byte(buf[i]);
    }
}

/* ── VHCI callbacks ─────────────────────────────────────────────────────── */

/* Called by the BT controller when it is ready to receive another packet. */
static void vhci_notify_send_available(void)
{
    xSemaphoreGive(s_send_avail_sem);
}

/* Called by the BT controller with a complete outbound HCI packet
 * (type byte is already the first byte of data).
 * We must NOT call TinyUSB write functions here — this runs inside the BT
 * controller task and write_flush would need to block waiting for the USB
 * endpoint, which is not allowed in a callback.  Queue instead. */
static int vhci_notify_recv(uint8_t *data, uint16_t len)
{
    if (len == 0 || len > MAX_HCI_PKT) {
        ESP_LOGW(TAG, "ctrl→host: ignoring packet with bad length %u", len);
        return 0;
    }

    hci_pkt_t pkt;
    memcpy(pkt.data, data, len);
    pkt.len = len;

    if (xQueueSend(s_from_vhci_q, &pkt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "ctrl→host queue full, packet dropped (type=0x%02x)", data[0]);
    }
    return 0;
}

static const esp_vhci_host_callback_t s_vhci_cb = {
    .notify_host_send_available = vhci_notify_send_available,
    .notify_host_recv           = vhci_notify_recv,
};

/* ── USB send task (ctrl→host) ──────────────────────────────────────────── */

/* Drains s_from_vhci_q and writes each packet to the CDC TX FIFO.
 * Runs as a regular FreeRTOS task so that write_flush can block until the
 * USB endpoint is free without violating callback constraints. */
static void usb_send_task(void *arg)
{
    (void)arg;
    hci_pkt_t pkt;

    for (;;) {
        if (xQueueReceive(s_from_vhci_q, &pkt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        esp_err_t err = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0,
                                                   pkt.data, pkt.len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "CDC write_queue: %s", esp_err_to_name(err));
            continue;
        }

        /* Block up to 200 ms for the USB endpoint to become free. */
        err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0,
                                         pdMS_TO_TICKS(200));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "CDC write_flush: %s", esp_err_to_name(err));
        }
    }
}

/* ── VHCI send task ─────────────────────────────────────────────────────── */

static void vhci_send_task(void *arg)
{
    (void)arg;
    hci_pkt_t pkt;

    for (;;) {
        /* Block until a packet is queued by the CDC RX path. */
        if (xQueueReceive(s_to_vhci_q, &pkt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Wait until the controller is ready to accept a packet.
         * Check first in case the semaphore was already given before we got
         * here; otherwise block until notify_host_send_available fires. */
        if (!esp_vhci_host_check_send_available()) {
            xSemaphoreTake(s_send_avail_sem, portMAX_DELAY);
        } else {
            /* Drain any stale "available" signals so the semaphore count
             * stays tidy, but don't block. */
            xSemaphoreTake(s_send_avail_sem, 0);
        }

        esp_vhci_host_send_packet(pkt.data, pkt.len);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t hci_bridge_init(void)
{
    s_to_vhci_q = xQueueCreate(QUEUE_DEPTH, sizeof(hci_pkt_t));
    if (!s_to_vhci_q) {
        return ESP_ERR_NO_MEM;
    }

    s_from_vhci_q = xQueueCreate(QUEUE_DEPTH, sizeof(hci_pkt_t));
    if (!s_from_vhci_q) {
        vQueueDelete(s_to_vhci_q);
        return ESP_ERR_NO_MEM;
    }

    s_send_avail_sem = xSemaphoreCreateBinary();
    if (!s_send_avail_sem) {
        vQueueDelete(s_to_vhci_q);
        vQueueDelete(s_from_vhci_q);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_vhci_host_register_callback(&s_vhci_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register VHCI callback: %s", esp_err_to_name(err));
        return err;
    }

    /* vhci_send_task: host→controller.  Pin to core 0 alongside BT controller. */
    BaseType_t rc = xTaskCreatePinnedToCore(
        vhci_send_task, "vhci_send", 4096, NULL, 5, NULL, 0);
    if (rc != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* usb_send_task: controller→host.  No core affinity needed. */
    rc = xTaskCreate(usb_send_task, "usb_send", 4096, NULL, 5, NULL);
    if (rc != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HCI bridge initialised");
    return ESP_OK;
}

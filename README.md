# ESP32-S3 BLE Adapter — HCI H:4 over USB CDC

Uses an **M5Stack Atom S3 Lite** (ESP32-S3) as a BLE-only Bluetooth adapter
for a Linux PC via BlueZ (`bluetoothctl` / `btmgmt`).

The firmware bridges the ESP32-S3's built-in BLE controller (via the VHCI
API) to Linux over a USB CDC-ACM serial port, using the standard HCI H:4
framing.  Linux attaches it with `btattach`, creating a normal `hci0` device.
No custom kernel driver is needed.

```
Linux (BlueZ)
    │  HCI H:4 over USB CDC-ACM (/dev/ttyACM0)
    ▼
ESP32-S3 firmware  (this repo)
    │  VHCI API
    ▼
ESP32-S3 BLE controller
```

## Requirements

- [ESP-IDF v5.5.3](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/)
- Linux host with BlueZ (`bluez` package) and the `cdc_acm` kernel module

## Build & flash

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash   # or use esptool directly
```

## One-time Linux setup

### 1. Ensure the `cdc_acm` module is available

On Ubuntu minimal/cloud images the module ships in a separate package:

```bash
sudo apt-get install linux-modules-extra-$(uname -r)
sudo modprobe cdc_acm
```

To load it automatically at boot:

```bash
echo cdc_acm | sudo tee /etc/modules-load.d/cdc_acm.conf
```

### 2. Udev rule — stable device name

Create `/etc/udev/rules.d/99-esp32-bt.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", \
  SYMLINK+="esp32bt", TAG+="systemd", ENV{SYSTEMD_WANTS}="btattach-esp32.service"
```

This matches the Espressif TinyUSB CDC VID/PID, creates a stable
`/dev/esp32bt` symlink, and tells systemd to start the attach service
whenever the device appears (at boot or on hotplug).

### 3. Systemd service — auto-attach on plug-in

Create `/etc/systemd/system/btattach-esp32.service`:

```ini
[Unit]
Description=HCI bridge – ESP32-S3 BLE adapter
After=dev-esp32bt.device
BindsTo=dev-esp32bt.device

[Service]
ExecStart=/usr/bin/btattach -B /dev/esp32bt -P h4 -S 115200
Restart=on-failure
RestartSec=3

[Install]
WantedBy=dev-esp32bt.device
```

`BindsTo=` stops the service cleanly when the device is unplugged.

### 4. Activate

```bash
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl enable btattach-esp32.service
```

Unplug and replug the device, then verify:

```bash
ls -la /dev/esp32bt
systemctl status btattach-esp32
btmgmt info
```

## Manual bring-up (without auto-attach)

```bash
sudo modprobe cdc_acm          # if not already loaded
sudo btattach -B /dev/ttyACM0 -P h4 -S 115200 &
btmgmt info
bluetoothctl
```

## Using BlueZ

```bash
bluetoothctl
> power on
> scan on

# or
sudo btmgmt find
sudo hcitool lescan
```

## How it works

| Direction | Path |
|-----------|------|
| Host → controller | Linux writes H:4 bytes → CDC RX callback → H:4 reassembly → `esp_vhci_host_send_packet()` |
| Controller → host | `notify_host_recv()` callback → queue → `usb_send_task` → `tinyusb_cdcacm_write_queue()` + `write_flush()` → USB IN endpoint |

H:4 packet-type bytes (included as byte 0 by both Linux and the VHCI API):

| Byte | Type |
|------|------|
| `0x01` | HCI Command (host → controller) |
| `0x02` | ACL Data (both directions) |
| `0x04` | HCI Event (controller → host) |

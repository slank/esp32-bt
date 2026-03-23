# ESP32-S3 BLE Adapter — HCI H:4 over USB CDC

Uses an **M5Stack Atom S3 Lite** (ESP32-S3) as a BLE-only Bluetooth adapter
for a Linux PC via BlueZ (`bluetoothctl` / `btmgmt`).

The firmware bridges the ESP32-S3's built-in BLE controller (via the VHCI API)
to Linux over USB CDC-ACM using standard HCI H:4 framing. Linux attaches it
with `btattach`, creating a normal `hci0` device. No custom kernel driver needed.

This project is licensed under Apache-2.0. See `LICENSE` for details.

## Flash firmware

**Easiest:** use the [web flasher](https://slank.github.io/esp32-bt/) in Chrome or Edge — no toolchain required.

**Manually:** download the firmware bundle from [Releases](https://github.com/slank/esp32-bt/releases) and flash with `esptool.py`, or build from source:

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Full build/flash and prebuilt firmware instructions are in the [wiki](https://github.com/slank/esp32-bt/wiki), which also covers manual bring-up, BlueZ usage, and how it works.

## Linux setup

After flashing, see the wiki for one-time host configuration:

- [systemd (Debian/Ubuntu)](https://github.com/slank/esp32-bt/wiki/Setup-Systemd)
- [Alpine Linux / OpenRC](https://github.com/slank/esp32-bt/wiki/Setup-Alpine)

## Development

Requires [ESP-IDF v5.5.3](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/).

Clone with the wiki submodule:

```bash
git clone --recurse-submodules https://github.com/slank/esp32-bt.git
```

Source is in `main/`. The entry point is `main.c`; HCI bridge logic is in `hci_bridge.c`.

## License

Apache License, Version 2.0. Third-party component versions are in `dependencies.lock`.

# Contributing

## Development setup

1. Install ESP-IDF v5.5.3 and export the environment.
2. Clone the repository and initialize the ESP-IDF shell for your platform.
3. Build locally with `idf.py build`.
4. Flash and validate on ESP32-S3 hardware with `idf.py -p /dev/ttyACM0 flash monitor`.

## Change expectations

- Keep changes narrowly scoped and preserve the existing ESP-IDF coding style.
- Prefer root-cause fixes over one-off workarounds.
- Update `README.md` when behavior, flashing steps, or host setup instructions change.
- Update `CHANGELOG.md` for user-visible changes.
- Add SPDX license headers to new first-party source files.

## Validation

Before opening a pull request, run:

```bash
idf.py build
```

If your change affects host bring-up or USB transport behavior, also verify:

```bash
sudo btattach -B /dev/ttyACM0 -P h4 -S 115200
btmgmt info
```

## Pull requests

- Describe the problem and the behavioral change clearly.
- Include the hardware used for validation.
- Include the commands you ran and the result.
- Keep generated build output out of commits.
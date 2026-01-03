# Presence One Firmware
This is the firmware for Presence One, an open-source mmWave presence sensor using Matter over Thread.

## Prerequisites

Install the [nRF Connect SDK v3.2.1](https://docs.nordicsemi.com/bundle/ncs-3.2.1/page/nrf/installation/install_ncs.html), the rest of this document assumes you're using the command line installation with `nrfutil`.

## Building
Use `west` to set up the workspace, don't clone this repo manually unless you know what you're doing.

```
nrfutil sdk-manager toolchain launch --ncs-version v3.2.1 --shell
west init -m git@github.com:LocalizedTech/presence-one-firmware.git presence-one
cd presence-one
west update
west build --no-sysbuild -b xiao_ble application
```

This will create a workspace with the following structure:

```
presence-one/
  .west/        -- West workspace
  application/  -- This repo
  external/     -- Zephyr and nRF Connect SDK
  build/        -- Build artifacts
```

## Flashing

1. Connect the Presence One to your PC using a USB-C cable
2. Double-click the reset button
3. Your PC should detect a USB drive named "XIAO-SENSE"
4. Copy and paste `build/zephyr/zephyr.uf2` to this drive
5. Wait until the copy is done and the drive disappears from your PC
6. Flashing is done, you can now disconnect the USB-C cable

## Licensing

This repository uses per-file SPDX license identifiers (see the `SPDX-License-Identifier:` header in each source/config file). The authoritative license texts live in this directory:

- `Apache-2.0` license text: `LICENSES/Apache-2.0.txt`
- `LicenseRef-Nordic-5-Clause` license text: `LICENSES/LicenseRef-Nordic-5-Clause.txt`

If a file does not contain an SPDX header, its licensing is ambiguous.

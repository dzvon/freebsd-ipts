# freebsd-ipts

FreeBSD kernel drivers for the touchscreen and the pen of the Microsoft
Surface Pro 7.

The Surface Pro 7 uses Intel Precise Touch & Stylus (IPTS). The touch
controller sends its data through an Intel Management Engine Interface
(MEI) device that is only for touch. FreeBSD has no driver for this
device, so the touchscreen does not work. This module adds the drivers.

## Status

| Function | Status |
|----------|--------|
| Touchscreen, one finger | Works (`hmt(4)`) |
| Pen | The device attaches (`hpen(4)`). Not tested. |
| Touchscreen, more than one finger | Not supported yet |
| Raw HID reports (heatmaps) | Available through `hidraw(4)` |

## How it works

The module contains two drivers:

- `mei`: a driver for the MEI "iTouch" device (`8086:34e4`). It resets
  the interface, starts the Host Bus Message (HBM) protocol, and lists
  the ME clients. It adds a child device for each client and gives the
  child drivers an API to send and receive messages.
- `ipts`: a driver for the IPTS client. It gives the touch controller
  a set of DMA buffers and receives the data frames. It is a HID
  transport: it adds a `hidbus(4)` child and gives it the HID reports.

The controller runs in single-touch mode. In this mode, the firmware
finds the position of one finger and sends it in report `0x40`. This
report is not in the HID descriptor of the device. The driver adds its
own descriptor for it and changes the report into a one-contact
multitouch report, so that `hmt(4)` attaches as a touchscreen.

## Supported hardware

| Device | MEI device | Status |
|--------|------------|--------|
| Surface Pro 7 | Intel Ice Lake iTouch (`8086:34e4`) | Tested |

Other Surface devices with IPTS can work, but they need their PCI ID in
`mei_probe()`.

## Requirements

- FreeBSD 15 (amd64). The module was tested on 15.1-STABLE.
- The kernel source tree that matches the running kernel.

## Build

1. Get the kernel source that matches the running kernel, for example
   in `/usr/src`.
2. Build the module:

   ```sh
   make SYSDIR=/usr/src/sys
   ```

## Install

1. Load the module for a test:

   ```sh
   sudo kldload ./mei.ko
   ```

2. Make sure that the drivers found the touch controller:

   ```sh
   dmesg | grep -E 'mei0|ipts0|hmt|hpen'
   ```

3. Install the module:

   ```sh
   sudo install -o root -g wheel -m 444 mei.ko /boot/modules/
   sudo kldxref /boot/modules
   ```

4. Load the module at boot:

   ```sh
   sudo sysrc -f /boot/loader.conf mei_load=YES
   ```

CAUTION: Build the module again after each kernel upgrade, before you
restart. A module that does not match the kernel does not load.

If the system does not start because of this module, go to the loader
prompt (option 3 in the loader menu). Then type `disable-module mei` and
`boot`.

## Debug information

The drivers show their state in sysctls:

- `dev.mei.0.clients`: the ME clients and their message counters
- `dev.mei.0.regs`: the MEI registers
- `dev.ipts.0.frames`, `dev.ipts.0.errors`: the data frame counters
- `dev.ipts.0.hid_descriptor`: the HID descriptor of the device
- `dev.ipts.0.dump`: set it to N to print the next N frames to `dmesg`

## Known problems

- Only one finger works. Multitouch needs the multitouch mode of the
  controller and processing of the heatmaps in userland, as
  [iptsd](https://github.com/linux-surface/iptsd) does on Linux.
- Output reports to the device (HID-to-ME) are not supported.
- The driver uses event mode only. Poll mode (`hw.ipts.mode=1`) is not
  tested.

## References

- The MEI documentation in the Linux kernel:
  `Documentation/driver-api/mei/`.
- The linux-surface project: <https://github.com/linux-surface>.

## License

BSD 2-Clause. See the `LICENSE` file.

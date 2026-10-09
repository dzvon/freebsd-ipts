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
| Touchscreen, one finger | Works without iptsd (`hmt(4)`) |
| Touchscreen, more than one finger | Works with [iptsd](#multitouch-with-iptsd) |
| Pen | The devices attach (`hpen(4)` and iptsd). Not tested. |
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

The `ipts` driver also sends feature and output reports to the device
through the HID-to-ME buffer. A program uses these reports to switch
the controller between single-touch mode and multitouch mode.

Without iptsd, the controller runs in single-touch mode. In this mode, the firmware
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

## Multitouch with iptsd

In multitouch mode, the controller sends raw capacitance data
(heatmaps), and a program in userland finds the fingers. On Linux,
[iptsd](https://github.com/linux-surface/iptsd) does this. The
`freebsd` branch of the fork
[dzvon/iptsd](https://github.com/dzvon/iptsd/tree/freebsd) adds
FreeBSD support and an rc.d service. iptsd uses the GPL-2.0, so it is
not part of this repository.

1. Install the build tools:

   ```sh
   sudo pkg install meson ninja pkgconf
   ```

2. Build iptsd. Meson downloads the libraries:

   ```sh
   git clone -b freebsd https://github.com/dzvon/iptsd.git
   cd iptsd
   meson setup build --wrap-mode=forcefallback -Dservice_manager=[] \
       -Ddebug_tools=[]
   ninja -C build src/iptsd
   ```

   One library runs a script with `python3`. If the system has no
   `python3` command, make a link to the installed Python, for example
   `python3.11`, in a directory in `PATH`.

3. Install iptsd and its service:

   ```sh
   sudo install -m 555 build/src/iptsd /usr/local/bin/iptsd
   sudo install -m 555 etc/freebsd/iptsd-run /usr/local/libexec/iptsd-run
   sudo install -m 555 etc/freebsd/rc.d/iptsd /usr/local/etc/rc.d/iptsd
   ```

4. Load `hidraw(4)` at boot, and start the service:

   ```sh
   sudo sysrc -f /boot/loader.conf hidraw_load=YES
   sudo kldload hidraw
   sudo sysrc iptsd_enable=YES
   sudo service iptsd start
   ```

iptsd makes the devices "IPTSD Virtual Touchscreen" and "IPTSD Virtual
Stylus". When iptsd stops, it switches the controller back to
single-touch mode.

## Debug information

The drivers show their state in sysctls:

- `dev.mei.0.clients`: the ME clients and their message counters
- `dev.mei.0.regs`: the MEI registers
- `dev.ipts.0.frames`, `dev.ipts.0.errors`: the data frame counters
- `dev.ipts.0.hid_descriptor`: the HID descriptor of the device
- `dev.ipts.0.report_ids`: the input reports for each report ID
- `dev.ipts.0.dump`: set it to N to print the next N frames to `dmesg`

The tool in `tools/iptsctl.c` reads and writes feature reports through
`hidraw(4)`. For example, `iptsctl get 0x05 2` reads the mode (0 is
single-touch, 1 is multitouch).

## Known problems

- Without iptsd, only one finger works.
- The driver uses event mode only. Poll mode (`hw.ipts.mode=1`) is not
  tested.

## References

- The MEI documentation in the Linux kernel:
  `Documentation/driver-api/mei/`.
- The linux-surface project: <https://github.com/linux-surface>.

## License

BSD 2-Clause. See the `LICENSE` file.

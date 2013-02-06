#!/usr/bin/env python
#
# stm32_mem.py: STM32 memory access using USB DFU class
# Copyright (C) 2011  Black Sphere Technologies
# Written by Gareth McMullin <gareth@blacksphere.co.nz>
# Modified by Felix Ruess <felix.ruess@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

from time import sleep
import struct
from sys import stdout, argv
from os import path

from optparse import OptionParser

import usb
import dfu

APP_ADDRESS = 0x08002000
SECTOR_SIZE = 2048

CMD_GETCOMMANDS = 0x00
CMD_SETADDRESSPOINTER = 0x21
CMD_ERASE = 0x41

def stm32_erase(dev, addr):
    erase_cmd = struct.pack("<BL", CMD_ERASE, addr)
    dev.download(0, erase_cmd)
    while True:
        status = dev.get_status()
        if status.bState == dfu.STATE_DFU_DOWNLOAD_BUSY:
            sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_DOWNLOAD_IDLE:
            break

def stm32_write(dev, data):
    dev.download(2, data)
    while True:
        status = dev.get_status()
        if status.bState == dfu.STATE_DFU_DOWNLOAD_BUSY:
            sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_DOWNLOAD_IDLE:
            break

def stm32_manifest(dev):
    dev.download(0, "")
    while True:
        try:
            status = dev.get_status()
        except:
            return
        sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_MANIFEST:
            break

def print_copyright():
    print("")
    print("USB Device Firmware Upgrade - Host Utility -- version 1.3")
    print("Copyright (C) 2011  Black Sphere Technologies")
    print("Copyright (C) 2012  Transition Robotics Inc.")
    print("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>")
    print("")

if __name__ == "__main__":
    usage = "Usage: %prog [options] firmware.bin" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage, version='%prog version 1.3')
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if path.isfile(args[0]):
            binfile = args[0]
        else:
            parser.error("Binary file " + args[0] + " not found")

    if options.verbose:
        print_copyright()

    devs = dfu.finddevs()
    if not devs:
        print("No devices found!")
        exit(-1)
    elif options.verbose:
        print("Found %i devices." % len(devs))

    valid_manufacturers = []
    valid_manufacturers.append("Transition Robotics Inc.")
    valid_manufacturers.append("STMicroelectronics")
    # don't accept BMP to not accidentally upload ap firmware to it
    # valid_manufacturers.append("Black Sphere Technologies")

    # stm32 (autopilot) device which is found
    stm32dev = None

    for dev in devs:
        try:
            dfudev = dfu.dfu_device(*dev)
        except:
            if options.verbose:
                print("Could not open DFU device %s ID %04x:%04x\n"
                      "Maybe the OS driver is claiming it?" %
                      (dev[0].filename, dev[0].idVendor, dev[0].idProduct))
            continue
        try:
            man = dfudev.handle.getString(dfudev.dev.iManufacturer, 30)
            product = dfudev.handle.getString(dfudev.dev.iProduct, 30)
            serial = dfudev.handle.getString(dfudev.dev.iSerialNumber, 40)
        except:
            print("Whoops... could not get device description.")
            continue

        if options.verbose:
            print("Found device %s: ID %04x:%04x %s - %s - %s" %
                    (dfudev.dev.filename, dfudev.dev.idVendor,
                     dfudev.dev.idProduct, man, product, serial))

        if man in valid_manufacturers:
            stm32dev = dfudev
            break

    if stm32dev is None:
        print("Could not find STM32 (autopilot) device.")
        exit(-1)

    print("Using device %s: ID %04x:%04x %s - %s - %s" % (stm32dev.dev.filename,
          stm32dev.dev.idVendor, stm32dev.dev.idProduct, man, product, serial))

    try:
        state = stm32dev.get_state()
    except:
        print("Failed to read device state! Assuming APP_IDLE")
        state = dfu.STATE_APP_IDLE
    if state == dfu.STATE_APP_IDLE:
        stm32dev.detach()
        print("Run again to upgrade firmware.")
        exit(0)

    stm32dev.make_idle()

    try:
        bin = open(binfile, "rb").read()
    except:
        print("Could not open binary file.")
        raise

    addr = APP_ADDRESS
    while bin:
        print("Programming memory at 0x%08X\r" % addr)
        stdout.flush()
        stm32_erase(stm32dev, addr)
        stm32_write(stm32dev, bin[:SECTOR_SIZE])

        bin = bin[SECTOR_SIZE:]
        addr += SECTOR_SIZE

    stm32_manifest(stm32dev)

    print("\nAll operations complete!\n")

#!/usr/bin/env bash

astra_usb_device() {
    local device bus_number device_number
    for device in /sys/bus/usb/devices/*; do
        [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
        if grep -qx '2bc5' "$device/idVendor" && grep -qx '0402' "$device/idProduct"; then
            bus_number="$(printf '%03d' "$((10#$(<"$device/busnum")))")"
            device_number="$(printf '%03d' "$((10#$(<"$device/devnum")))")"
            printf '/dev/bus/usb/%s/%s\n' "$bus_number" "$device_number"
            return 0
        fi
    done
    echo 'Orbbec Astra S USB device 2bc5:0402 was not found on the host.' >&2
    return 1
}

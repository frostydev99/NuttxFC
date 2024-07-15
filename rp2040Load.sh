#!/bin/bash

detect_rp2040_port() {
    
    # Detect RP2040 boards
    SERIAL_PORT=$(picotool list | grep 'tty' | awk '{print $3}')
    
    # Check if serial port is valid and accessible
    if [ -z "$SERIAL_PORT" ]; then
        echo "RP2040 serial port not found."
        return 1
    fi

    echo "Detected RP2040 serial port: $SERIAL_PORT"
    return 0
}

# Main script execution
detect_rp2040_port
if [ $? -eq 0 ]; then
    picotool reboot --usb
    sleep 4  # Wait a considerable time for pico to enter DFU
    upload_uf2_file
    picotool load ./nuttx/firmware.uf2
fi

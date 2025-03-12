#!/bin/bash

# Check if an argument was passed
if [ -z "$1" ]; then
    echo "Usage: $0 <udp_bw_value>"
    exit 1
fi

# Get the input value (e.g., udp_bw 2M)
udp_bw=$1

# Map the input value to the corresponding hex byte
case "$udp_bw" in
    "2M")
        hex_value="\x01"
        ;;
    "1M")
        hex_value="\x02"
        ;;
    "500k")
        hex_value="\x03"
        ;;
    "250k")
        hex_value="\x04"
        ;;
    *)
        echo "Invalid udp_bw value. Supported values are: 2M, 1M, 500k, 250k."
        exit 1
        ;;
esac

# Send the UDP message using netcat
echo -n -e "$hex_value" | nc -u -w1 127.0.0.1 23456

# Output the sent value
echo "Sent value: $hex_value"


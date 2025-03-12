#!/bin/bash

# Check if an argument was passed
if [ -z "$1" ]; then
    echo "Usage: $0 <value>"
    exit 1
fi

# Get the input value (udp_bw)
udp_bw=$1

# Convert the input value to hexadecimal format (e.g., 3 -> 0x03)
hex_value=$(printf "\\x%02x" "$udp_bw")

# Send the UDP message using netcat
echo -n -e "$hex_value" | nc -u -w1 127.0.0.1 23456

echo "Sent value: $hex_value"


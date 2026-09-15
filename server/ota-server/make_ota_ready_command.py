#!/usr/bin/env python3
"""Builds the checksummed EVENT_COMMAND payload for the OTA_READY push
signal (see teleclient.cpp's TeleClientUDP::inbound(), EVENT_COMMAND case).

Checksum matches both sides exactly: Traccar's Checksum.sum(String) (8-bit
sum of ASCII bytes, formatted as 2 uppercase hex digits) and the device's
TeleClientUDP::verifyChecksum() (identical algorithm) - verified by reading
both implementations side by side, not assumed.

Usage: python3 make_ota_ready_command.py <device_id_string>
  e.g. python3 make_ota_ready_command.py ZKUCA42T

Prints the exact string to paste into Traccar's Send Command dialog
(Type: Custom command, Data: <this string>), or to pass as the "data"
field of a POST /api/commands/send call.
"""
import sys
import time


def checksummed(message: str) -> str:
    checksum = sum(message.encode("ascii")) & 0xFF
    return f"{message}*{checksum:02X}"


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    device_id = sys.argv[1]
    ts = int(time.time() * 1000) % 100000000  # matches device's millis()-style TS field width
    message = f"EV=5,TS={ts},ID={device_id},CMD=OTA_READY"
    print(checksummed(message))


if __name__ == "__main__":
    main()

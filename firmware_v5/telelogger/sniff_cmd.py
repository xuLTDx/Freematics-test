#!/usr/bin/env python3
"""Switches the box's sniff mode (sniff.h) from the server, over whatever link
the box has (WiFi or cellular): a Traccar custom command, delivered the same
way as OTA_READY (queued by Traccar while the box is offline).

Usage:
  python3 sniff_cmd.py on [minutes]   # default 120, max 1440
  python3 sniff_cmd.py off
Options: --device ZKUCA42T (Traccar uniqueId / box ID),
         --creds /home/claude/traccar_admin_credentials.json

The box confirms in Traccar: attribute io915 (PID 0x393) = minutes left,
0 = off, and the SD event log has "SNIFF ON file=S<n>" / "SNIFF OFF".
"""
import argparse
import base64
import json
import time
import urllib.request


def checksummed(message):
    # same 8-bit sum as the box's verifyChecksum() and make_ota_ready_command.py
    return f"{message}*{sum(message.encode('ascii')) & 0xFF:02X}"


def main():
    ap = argparse.ArgumentParser(description="sniff mode on/off via Traccar")
    ap.add_argument("action", choices=["on", "off"])
    ap.add_argument("minutes", nargs="?", type=int, default=120)
    ap.add_argument("--device", default="ZKUCA42T")
    ap.add_argument("--creds", default="/home/claude/traccar_admin_credentials.json")
    a = ap.parse_args()
    if a.action == "on" and not 1 <= a.minutes <= 1440:
        ap.error("minutes: 1-1440")

    c = json.load(open(a.creds))
    base = c["url"].rstrip("/")
    auth = "Basic " + base64.b64encode(f"{c['email']}:{c['password']}".encode()).decode()

    def call(path, body=None):
        req = urllib.request.Request(base + path, headers={"Authorization": auth, "Content-Type": "application/json"},
                                     data=json.dumps(body).encode() if body is not None else None,
                                     method="POST" if body is not None else "GET")
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.read()

    dev = [d for d in json.loads(call("/api/devices")) if d["uniqueId"] == a.device]
    if not dev:
        raise SystemExit(f"device {a.device} not found in Traccar")
    cmd = f"SNIFF=1,{a.minutes}" if a.action == "on" else "SNIFF=0"
    ts = int(time.time() * 1000) % 100000000
    data = checksummed(f"EV=5,TS={ts},ID={a.device},CMD={cmd}")
    call("/api/commands/send", {"deviceId": dev[0]["id"], "type": "custom", "attributes": {"data": data}})
    print(f"sent {cmd} to {a.device} (status {dev[0]['status']}; queued by Traccar if offline)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Push-decision watcher for the Freematics pull-OTA server.

ota_server.py stays a completely passive responder: it never decides
anything, it just answers whatever a device asks. This script is the
opposite side - it decides, on its own schedule, whether a device is
actually due for an update, and if so tells the device to check *now*
instead of waiting for its next OTA_INTERVAL-timed poll.

How it decides, per registry.json entry (device_ip must be set - entries
without one are skipped, so this stays fully optional and backward
compatible with a registry.json written before this script existed):
  1. GET http://<device_ip>/api/info  - the device's own live, authoritative
     answer to "what firmware am I actually running right now" (fw_build,
     plus id - see the device_id check below). This is deliberately NOT
     inferred from Traccar's log: Traccar only ever sees telemetry packets,
     never a firmware version, so the device's own /api/info is the one
     source of truth for this.
  2. If this entry has a device_id, it MUST match /api/info's "id" field or
     the entry is refused (loud warning, nothing sent). We now have two
     genuinely different vehicle-specific firmwares in play (VAG/Passat vs
     PSA/Zafira, each with its own registry token/firmware path per the
     "one token maps to exactly one firmware file" rule in README.md) - a
     stale/wrong device_ip in the registry (device got a new DHCP lease,
     two entries swapped by a copy-paste mistake, etc.) would otherwise
     silently push one vehicle's firmware onto the other's hardware. This
     check is what actually catches that, since device_ip alone can't be
     trusted to still point at the device the entry thinks it does.
  3. Compare fw_build against this entry's target_build with the exact same
     is_update_needed() logic ota_server.py's own meta.json handler uses -
     imported from there, not reimplemented, so the two can never disagree
     about what counts as "needs an update".
  4. If needed and the device answered at all (i.e. it's on the LAN and
     reachable right now): GET http://<device_ip>/api/control?cmd=OTA_CHECK_NOW.
     That's the entire "push" - it just makes the device's own existing,
     already-hash-verified pull-OTA pipeline (stage to SD, verify SHA256,
     flash at next standby) run immediately rather than up to OTA_INTERVAL
     seconds from now. This script never touches SD, flash, or hashes
     itself - all of that stays exactly as already implemented on the
     device (performPullOtaCheck() / performPullOtaFlash() in telelogger.ino).

A device that's unreachable right now (not on this LAN, powered off,
mid-drive without WiFi) is simply skipped this pass - no error, no retry
storm, it's picked up again next pass whenever it's reachable.

Run directly (loops forever, sleeping PUSH_CHECK_INTERVAL_S between passes)
or under systemd (see freematics-ota-push.service).
"""
import json
import sys
import time
import urllib.request

from ota_server import load_registry, is_update_needed

HTTP_TIMEOUT_S = 5
PUSH_CHECK_INTERVAL_S = 300  # 5 minutes between passes


def device_info(ip):
    """GET http://<ip>/api/info, return the parsed JSON dict or None."""
    try:
        with urllib.request.urlopen(f"http://{ip}/api/info", timeout=HTTP_TIMEOUT_S) as r:
            return json.loads(r.read().decode("utf-8"))
    except Exception:
        return None


def trigger_check_now(ip):
    """GET http://<ip>/api/control?cmd=OTA_CHECK_NOW, return True on 'OK'."""
    try:
        with urllib.request.urlopen(
            f"http://{ip}/api/control?cmd=OTA_CHECK_NOW", timeout=HTTP_TIMEOUT_S
        ) as r:
            return r.read().decode("utf-8").strip() == "OK"
    except Exception:
        return False


def run_once():
    registry = load_registry()
    for token, entry in registry.items():
        label = entry.get("device", token[:8])
        if not entry.get("enabled"):
            continue
        ip = entry.get("device_ip")
        if not ip:
            continue  # registry entry not opted into push checking

        info = device_info(ip)
        if info is None:
            print(f"[{label}] {ip} unreachable, skipping")
            continue

        reported_build = info.get("fw_build")
        target_build = entry.get("target_build")
        if not is_update_needed(reported_build, target_build):
            print(f"[{label}] {ip} up to date ({reported_build})")
            continue

        print(f"[{label}] {ip} running {reported_build!r}, target {target_build!r} - triggering check")
        if trigger_check_now(ip):
            print(f"[{label}] OTA_CHECK_NOW accepted")
        else:
            print(f"[{label}] OTA_CHECK_NOW failed (device unreachable or rejected it)")


def main():
    loop = "--once" not in sys.argv
    while True:
        try:
            run_once()
        except Exception as e:
            print(f"[ota-push-watcher] pass failed: {e}", file=sys.stderr)
        if not loop:
            break
        time.sleep(PUSH_CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()

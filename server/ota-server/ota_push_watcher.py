#!/usr/bin/env python3
"""Push-decision watcher for the Freematics pull-OTA server.

ota_server.py stays a completely passive responder: it never decides
anything, it just answers whatever a device asks. This script is the
opposite side - it decides whether a device is due for an update, and if
so, tells it right now via Traccar's own command-delivery mechanism -
instead of the device asking "anything new?" on a timer.

Both steps below - reading a device's current firmware and telling it an
update is ready - go through Traccar's REST API, not a direct connection to
the device. This is deliberate, not just convenient: a device on cellular
data has no directly-reachable IP at all (it sits behind carrier-grade NAT
sharing a public IP with thousands of other SIM cards), so anything that
tries to open a NEW inbound connection to it - whether to query /api/info
or to push bytes - simply cannot work over cellular, only on a LAN with no
NAT in the way. Traccar's UDP session for this device is different: it
was opened by the DEVICE (outbound), so the NAT/firewall on its side keeps
that mapping's reverse path open for replies - which is exactly the path
Command.TYPE_CUSTOM rides down. See teleclient.cpp's TeleClientUDP::inbound()
EVENT_COMMAND case for the device-side receiver.

How it decides, per registry.json entry with a traccar_device_id set
(entries without one are skipped - stays optional/backward compatible):
  1. GET {traccar_url}/api/devices/{id} for its positionId, then
     GET {traccar_url}/api/positions?id={positionId} for that position's
     attributes.versionFw - the device's last self-reported build (sent
     once per LOGIN in teleclient.cpp's notify() payload, decoded into
     Position.KEY_VERSION_FW by FreematicsProtocolDecoder.java). This is a
     database-backed attribute, not a line in a rotatable log file, so it
     survives however long the device stays offline and however Traccar's
     own log rotation is configured.
  2. Compare against this entry's target_build with the exact same
     is_update_needed() logic ota_server.py's own meta.json handler uses -
     imported from there, not reimplemented.
  3. If needed: authenticate to Traccar (POST /api/session) and
     POST /api/commands/send with a Command.TYPE_CUSTOM whose data is the
     checksummed "EV=5,TS=...,ID=...,CMD=OTA_READY*XX" string (see
     make_ota_ready_command.py for the same checksum, kept in sync here).
     If the device isn't in a live session right now (offline, mid-drive
     with no signal, parked for months), Traccar queues the command in its
     own database (tc_commands_queue, no TTL) and delivers it automatically
     the moment the device's next packet is decoded - verified by reading
     CommandsManager/ExtendedObjectDecoder directly, not assumed. Nothing
     about that queuing is specific to this protocol or this script.
  4. The device's own EVENT_COMMAND handler then makes its existing pull-OTA
     check (performPullOtaCheck() - SD-staged download, incremental SHA256
     against meta.json's hash, only written to the trusted marker file on a
     match) run immediately. This script never touches SD/flash/hashes.

Run directly (loops forever, sleeping PUSH_CHECK_INTERVAL_S between passes)
or under systemd (see freematics-ota-push.service). In practice this should
be triggered right after publishing a new build (see the "event-driven, not
blind polling" note below) rather than left on a long timer.
"""
import json
import logging
import logging.handlers
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

from ota_server import load_registry, is_update_needed

HTTP_TIMEOUT_S = 8
PUSH_CHECK_INTERVAL_S = 300  # fallback only - see module docstring
CREDENTIALS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "traccar_credentials.json")
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ota_push_watcher.log")

log = logging.getLogger("ota_push_watcher")
log.setLevel(logging.INFO)
_console = logging.StreamHandler(sys.stdout)  # -> journalctl under systemd
_file = logging.handlers.RotatingFileHandler(LOG_PATH, maxBytes=2_000_000, backupCount=3)
_fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
for _h in (_console, _file):
    _h.setFormatter(_fmt)
    log.addHandler(_h)


def load_credentials():
    with open(CREDENTIALS_PATH, "r", encoding="utf-8-sig") as f:
        return json.load(f)


def checksummed(message: str) -> str:
    """Same algorithm on both ends: Traccar's Checksum.sum(String) and the
    device's TeleClientUDP::verifyChecksum() - verified side by side, not
    assumed. 8-bit sum of ASCII bytes, 2 uppercase hex digits."""
    checksum = sum(message.encode("ascii")) & 0xFF
    return f"{message}*{checksum:02X}"


def ota_ready_command(device_id_str: str) -> str:
    ts = int(time.time() * 1000) % 100000000
    return checksummed(f"EV=5,TS={ts},ID={device_id_str},CMD=OTA_READY")


class TraccarSession:
    """Cookie-authenticated client for the handful of Traccar REST calls
    this script needs. Logs in once per run, not per request."""

    def __init__(self, base_url, email, password):
        self.base_url = base_url.rstrip("/")
        self.jar = urllib.request.HTTPCookieProcessor()
        self.opener = urllib.request.build_opener(self.jar)
        data = urllib.parse.urlencode({"email": email, "password": password}).encode()
        req = urllib.request.Request(f"{self.base_url}/api/session", data=data, method="POST")
        self.opener.open(req, timeout=HTTP_TIMEOUT_S).read()

    def get(self, path):
        req = urllib.request.Request(f"{self.base_url}{path}")
        with self.opener.open(req, timeout=HTTP_TIMEOUT_S) as r:
            return json.loads(r.read().decode("utf-8"))

    def post_json(self, path, obj):
        data = json.dumps(obj).encode("utf-8")
        req = urllib.request.Request(
            f"{self.base_url}{path}", data=data, method="POST",
            headers={"Content-Type": "application/json"},
        )
        with self.opener.open(req, timeout=HTTP_TIMEOUT_S) as r:
            return r.read()


def current_build(session, traccar_device_id):
    """Latest self-reported fw_build for this device, or None if it has
    never sent one (e.g. never logged in since the FW= payload was added)."""
    device = session.get(f"/api/devices/{traccar_device_id}")
    position_id = device.get("positionId")
    if not position_id:
        return None
    position = session.get(f"/api/positions?id={position_id}")
    if isinstance(position, list):
        position = position[0] if position else {}
    return (position.get("attributes") or {}).get("versionFw")


def push_ota_ready(session, traccar_device_id, device_id_str):
    session.post_json("/api/commands/send", {
        "deviceId": traccar_device_id,
        "type": "custom",
        "attributes": {"data": ota_ready_command(device_id_str)},
    })


def run_once(session):
    registry = load_registry()
    for token, entry in registry.items():
        label = entry.get("device", token[:8])
        if not entry.get("enabled"):
            continue
        traccar_device_id = entry.get("traccar_device_id")
        device_id_str = entry.get("device_id")
        if not traccar_device_id or not device_id_str:
            continue  # registry entry not opted into push checking

        reported_build = current_build(session, traccar_device_id)
        target_build = entry.get("target_build")
        if not is_update_needed(reported_build, target_build):
            log.info("[%s] up to date (%s)", label, reported_build)
            continue

        log.info("[%s] running %r, target %r - pushing OTA_READY", label, reported_build, target_build)
        try:
            push_ota_ready(session, traccar_device_id, device_id_str)
            log.info("[%s] command accepted (delivered now, or queued if currently offline)", label)
        except urllib.error.HTTPError as e:
            log.error("[%s] command send failed: HTTP %s %s", label, e.code, e.reason)


def main():
    creds = load_credentials()
    session = TraccarSession(creds["url"], creds["email"], creds["password"])
    loop = "--once" not in sys.argv
    log.info("ota_push_watcher starting (loop=%s)", loop)
    while True:
        try:
            run_once(session)
        except Exception:
            log.exception("pass failed")
        if not loop:
            break
        time.sleep(PUSH_CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()

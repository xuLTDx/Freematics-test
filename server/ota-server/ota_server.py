#!/usr/bin/env python3
"""Minimal pull-OTA server for Freematics telelogger devices.

Serves the exact API telelogger.ino's checkPullOta() expects:
  GET /api/freematics/ota_pull/<token>/meta.json
  GET /api/freematics/ota_pull/<token>/firmware.bin

Each token maps to one device + one specific firmware file via registry.json,
kept deliberately dumb and manual: nothing is served unless a registry entry
explicitly has "enabled": true, and each token is tied to exactly one
firmware file - this is the mechanism that keeps different vehicle profiles
(e.g. VAG-specific UDS code vs a future Stellantis build) from ever being
cross-served to the wrong device, since a device's token can only ever
resolve to the one firmware file its registry entry names.

Registry format (registry.json, same directory - not committed, see
registry.example.json):
{
  "<token-hex>": {
    "device": "human-readable label, e.g. ZKUCA42T (Passat B8, VAG)",
    "firmware": "/absolute/path/to/firmware.bin",
    "target_build": "Sep 14 2026 12:04:32",
    "enabled": false
  },
  ...
}

size/sha256 are computed on the fly from the firmware file (small enough -
~1.6MB - that hashing per request is not worth caching).

Self-regulating updates: the device reports its currently-running build
(telelogger.ino's __DATE__ " " __TIME__) as a ?build= query param on every
meta.json check. The reported build and this entry's target_build are both
parsed as timestamps and compared - available:true only when target_build
is strictly NEWER than what the device already reports. This is deliberate,
not just an equality check: `enabled` can be left permanently true once a
device is provisioned, and a new build only needs its target_build + path
updated here - but it also means a stale/older target_build (e.g. left over
from testing, or a registry not updated after a fresh local build) can never
push the device backwards, even if the strings simply don't match. Equal or
newer-than-target reported builds get available:false. If either timestamp
fails to parse, falls back to a plain equality check (fails safe: treats an
unparseable pair as "no update" rather than risking a downgrade).
`enabled: false` remains a hard kill switch regardless of target_build.

Deployment note: runs on a non-privileged port (8443 by default) rather than
443, since binding <1024 needs root and this is meant to run as an
unprivileged user. Point the device at it via the OTA_PORT= control command
(added to dataserver.cpp alongside this).
"""
import hashlib
import http.server
import json
import os
import re
import ssl
import sys
from datetime import datetime
from urllib.parse import urlsplit, parse_qs

REGISTRY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registry.json")
LISTEN_PORT = 8443

# C's __DATE__ " " __TIME__ (e.g. "Sep 14 2026 14:35:40") - single-digit days
# come through as "Sep  4 2026" (double space), so collapse whitespace first.
_BUILD_FMT = "%b %d %Y %H:%M:%S"


def parse_build(build_str):
    """Parse a "Mmm d(d) yyyy HH:MM:SS" build string, or None if unparseable."""
    if not build_str:
        return None
    try:
        return datetime.strptime(re.sub(r"\s+", " ", build_str.strip()), _BUILD_FMT)
    except ValueError:
        return None


def is_update_needed(reported_build, target_build):
    """True only if target_build is a strictly newer timestamp than reported_build.

    Falls back to plain string equality when either side fails to parse as a
    timestamp - so an update is only ever considered "needed" when we can
    positively confirm target is newer, never on ambiguous/malformed input.
    """
    reported_dt = parse_build(reported_build)
    target_dt = parse_build(target_build)
    if reported_dt is not None and target_dt is not None:
        return target_dt > reported_dt
    return reported_build != target_build


def load_registry():
    with open(REGISTRY_PATH, "r", encoding="utf-8-sig") as f:
        return json.load(f)


class Handler(http.server.BaseHTTPRequestHandler):
    def _not_found(self):
        self.send_response(404)
        self.end_headers()

    def _parse_path(self):
        # /api/freematics/ota_pull/<token>/<meta.json|firmware.bin>[?...]
        split = urlsplit(self.path)
        parts = split.path.strip("/").split("/")
        if len(parts) != 5 or parts[0:3] != ["api", "freematics", "ota_pull"]:
            return None, None, {}
        query = parse_qs(split.query)
        return parts[3], parts[4], query

    def do_GET(self):
        token, tail, query = self._parse_path()
        if token is None:
            self._not_found()
            return

        registry = load_registry()
        entry = registry.get(token)
        reported_build = query.get("build", [None])[0]

        if tail == "meta.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            if not entry or not entry.get("enabled"):
                body = json.dumps({"available": False}).encode()
            elif not is_update_needed(reported_build, entry.get("target_build")):
                # Device already reports running this entry's target build,
                # or (the important case) a build that's the same age or
                # NEWER than target_build - never offer to move it backwards.
                body = json.dumps({"available": False}).encode()
            else:
                fw_path = entry["firmware"]
                if not os.path.isfile(fw_path):
                    print(f"[WARN] registry entry for {entry.get('device')} points at missing file: {fw_path}", file=sys.stderr)
                    body = json.dumps({"available": False}).encode()
                else:
                    size = os.path.getsize(fw_path)
                    sha = hashlib.sha256()
                    with open(fw_path, "rb") as f:
                        for chunk in iter(lambda: f.read(65536), b""):
                            sha.update(chunk)
                    body = json.dumps({
                        "available": True,
                        "size": size,
                        "sha256": sha.hexdigest(),
                    }).encode()
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if tail == "firmware.bin":
            if not entry or not entry.get("enabled"):
                self._not_found()
                return
            fw_path = entry["firmware"]
            if not os.path.isfile(fw_path):
                self._not_found()
                return
            size = os.path.getsize(fw_path)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(size))
            self.end_headers()
            with open(fw_path, "rb") as f:
                while True:
                    chunk = f.read(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            return

        self._not_found()

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


def main():
    cert = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cert.pem")
    key = os.path.join(os.path.dirname(os.path.abspath(__file__)), "key.pem")
    server = http.server.HTTPServer(("0.0.0.0", LISTEN_PORT), Handler)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(certfile=cert, keyfile=key)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)
    print(f"OTA server listening on :{LISTEN_PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()

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
    "enabled": false
  },
  ...
}

size/sha256 are computed on the fly from the firmware file (small enough -
~1.6MB - that hashing per request is not worth caching).

Deployment note: runs on a non-privileged port (8443 by default) rather than
443, since binding <1024 needs root and this is meant to run as an
unprivileged user. Point the device at it via the OTA_PORT= control command
(added to dataserver.cpp alongside this).
"""
import hashlib
import http.server
import json
import os
import ssl
import sys

REGISTRY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registry.json")
LISTEN_PORT = 8443


def load_registry():
    with open(REGISTRY_PATH, "r", encoding="utf-8-sig") as f:
        return json.load(f)


class Handler(http.server.BaseHTTPRequestHandler):
    def _not_found(self):
        self.send_response(404)
        self.end_headers()

    def _parse_path(self):
        # /api/freematics/ota_pull/<token>/<meta.json|firmware.bin>
        parts = self.path.strip("/").split("/")
        if len(parts) != 5 or parts[0:3] != ["api", "freematics", "ota_pull"]:
            return None, None
        return parts[3], parts[4]

    def do_GET(self):
        token, tail = self._parse_path()
        if token is None:
            self._not_found()
            return

        registry = load_registry()
        entry = registry.get(token)

        if tail == "meta.json":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            if not entry or not entry.get("enabled"):
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

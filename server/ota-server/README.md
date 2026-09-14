# Freematics pull-OTA server

Minimal server implementing the pull-OTA API `telelogger.ino`'s `checkPullOta()`
already expects (built for a Home Assistant integration originally - see
`custom_components/freematics/`, not used by this deployment).

## Setup

1. `cp registry.example.json registry.json` and fill in one entry per device:
   a random 64-hex-char token, the device's human-readable name, and the
   absolute path to the firmware `.bin` that device should receive.
   `registry.json` is gitignored - it holds device secrets, never commit it.
2. Generate a self-signed TLS cert (the device never validates it -
   `client.setInsecure()` in `FreematicsNetwork.cpp` - so self-signed is fine):
   ```
   openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 3650 -nodes -subj '/CN=freematics-ota'
   ```
3. Run directly (`python3 ota_server.py`) or install `freematics-ota.service`
   as a systemd unit for persistence across reboots:
   ```
   sudo cp freematics-ota.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now freematics-ota
   ```
4. On the device, provision via its local `/api/control` endpoint (needs the
   device on WiFi):
   ```
   OTA_TOKEN=<the device's token>
   OTA_HOST=<this server's hostname/IP>
   OTA_PORT=8443
   OTA_INTERVAL=<seconds between checks, e.g. 3600>
   ```
5. Flip that device's registry entry to `"enabled": true` only when you
   actually want it to receive that firmware - the device will download and
   flash it (rebooting) on its next check. Leave `enabled: false` otherwise;
   this is the safe default and nothing auto-flashes without it.

## Why one token maps to exactly one firmware file

This is deliberate, not incidental: a device's token can only ever resolve
to the one firmware file its own registry entry names. When a second
vehicle platform needs different firmware (e.g. different UDS DIDs/module
addressing for a non-VAG vehicle), give it its own token and its own
registry entry pointing at its own `.bin` - never repoint an existing
device's token at a different vehicle's build.

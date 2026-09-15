# Freematics pull-OTA server

Minimal server implementing the pull-OTA API `telelogger.ino`'s `checkPullOta()`
already expects (built for a Home Assistant integration originally - see
`custom_components/freematics/`, not used by this deployment).

## Setup

### Option A: install the .deb package (recommended)

```
cd packaging && ./build_deb.sh && sudo dpkg -i freematics-ota_1.0.0_all.deb
```

This installs both scripts to `/opt/freematics-ota`, both systemd units, and
creates a dedicated `freematics-ota` system user. It does NOT create
`registry.json` or the TLS cert - the postinst step prints the exact
commands (same as steps 1-2 below, just rooted at `/opt/freematics-ota`).
Then `sudo systemctl enable --now freematics-ota freematics-ota-push`.

Upgrading a server that was set up manually (Option B, before this package
existed) to the package: stop the old unit, move `registry.json`/`cert.pem`/
`key.pem` from wherever they were (e.g. `/home/ultd/ota-server`) into
`/opt/freematics-ota`, `chown freematics-ota:freematics-ota` them, install
the .deb, then enable the new units - the units now run as the
`freematics-ota` user out of `/opt/freematics-ota`, not the old path/user.

### Option B: run the scripts directly

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
5. Set `"enabled": true` once the device is provisioned - this can stay
   `true` permanently. Whenever you build a new firmware:
   - Copy the new `.bin` over the path this entry's `firmware` field names.
   - Set `target_build` to the new build's exact `__DATE__ " " __TIME__`
     string (printed on serial boot as `FW:... Built:...`, or in
     `/api/info`'s `fw_build`).
   The device reports its currently-running build on every check (`?build=`
   query param) - once it matches `target_build`, the server automatically
   stops offering the update, no manual re-disable needed. `enabled: false`
   is still a hard kill switch if you ever need to pause OTA for a device
   entirely (e.g. mid-drive).

## Optional: push instead of waiting for the poll (ota_push_watcher.py)

By itself, `ota_server.py` is purely passive - a device only finds out about
an update on its next periodic poll (up to `OTA_INTERVAL` seconds away).
`ota_push_watcher.py` is a separate, optional process that decides *when* a
device is actually due and tells it to check immediately:

1. For each `registry.json` entry that has a `device_ip` set, it queries that
   device's own `http://<device_ip>/api/info` - the device's live,
   authoritative answer for what firmware it's actually running right now.
2. If the entry also has a `device_id`, it must match `/api/info`'s `"id"`
   field or the entry is refused with a warning. This matters once you have
   more than one vehicle profile (e.g. VAG/Passat and PSA/Zafira each have
   their own token + firmware file, per the rule below) - it's what stops a
   stale/wrong `device_ip` from silently pushing one vehicle's firmware onto
   another's hardware.
3. Compares the reported build against `target_build` with the exact same
   logic `ota_server.py`'s own `meta.json` handler uses.
4. If an update is due: `GET http://<device_ip>/api/control?cmd=OTA_CHECK_NOW`.
   That's the entire "push" - it just makes the device run its existing,
   already-hash-verified pull-OTA pipeline right away instead of waiting.

Add `device_ip` (and, once you have more than one vehicle profile,
`device_id`) to a `registry.json` entry to opt it into this - entries
without `device_ip` are left alone (still work via periodic polling only).
A device that's unreachable when checked (not on this LAN right now, no
WiFi) is just skipped and retried on the next pass - no error.

Run it the same two ways as `ota_server.py` (Option A's package installs
both units; for Option B, `python3 ota_push_watcher.py`, or add `--once` to
run a single pass instead of looping).

## Why one token maps to exactly one firmware file

This is deliberate, not incidental: a device's token can only ever resolve
to the one firmware file its own registry entry names. When a second
vehicle platform needs different firmware (e.g. different UDS DIDs/module
addressing for a non-VAG vehicle), give it its own token and its own
registry entry pointing at its own `.bin` - never repoint an existing
device's token at a different vehicle's build.

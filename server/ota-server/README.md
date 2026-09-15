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
device is actually due and tells it to check immediately - over cellular or
WiFi, wherever the device currently is, not just while it's reachable on
this LAN (see the module docstring for the full NAT/CGNAT reasoning: a
device on cellular has no directly-reachable IP at all, so both steps below
go through Traccar's own REST API and its already-open reverse path to the
device, never a new connection this script opens itself):

1. `cp traccar_credentials.example.json traccar_credentials.json` and fill
   in Traccar's URL and a login (a dedicated low-privilege user is fine -
   it only needs to read devices/positions and send commands).
   `traccar_credentials.json` is gitignored, same as `registry.json`.
2. Add `traccar_device_id` (Traccar's own numeric device ID - visible in its
   UI or via `GET /api/devices`) to a `registry.json` entry to opt it into
   this; entries without one are left alone (still work via periodic
   polling only).
3. For each opted-in entry, the watcher reads that device's last
   self-reported firmware build from Traccar (`Position.attributes.versionFw`,
   set once per LOGIN by `teleclient.cpp`'s `notify()` payload and decoded by
   `FreematicsProtocolDecoder.java` - durable in Traccar's database, not a
   line in a rotatable log file) and compares it to `target_build` with the
   same logic `ota_server.py`'s own `meta.json` handler uses.
4. If due: `POST /api/commands/send` (Traccar's own command-dispatch API, a
   `type: "custom"` command whose `data` is a checksummed
   `"EV=5,TS=...,ID=...,CMD=OTA_READY*XX"` string - see
   `make_ota_ready_command.py` for a standalone way to build the same string
   for manual testing via Traccar's web UI). Traccar delivers it immediately
   if the device has a live session, or queues it in its own database
   (indefinitely, no TTL - verified by reading `CommandsManager` directly)
   for automatic delivery the moment the device's next packet is decoded,
   however long that takes. The device's `TeleClientUDP::inbound()`
   `EVENT_COMMAND` case then triggers the same immediate check that
   `/api/control?cmd=OTA_CHECK_NOW` does locally - the rest of the pipeline
   (SD-staged download, SHA256 verify, flash at next standby) is unchanged.

Logs to `ota_push_watcher.log` (rotated, 3x2MB) next to the script, and to
stdout/journalctl under systemd.

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

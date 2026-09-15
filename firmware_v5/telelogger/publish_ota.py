"""PlatformIO post-build script: after a successful build, publish the
firmware to the OTA server automatically - no manual scp / registry.json
editing. This is the missing "point 1" of the OTA-push design: build here,
it goes to the server + registers as the new target_build, on its own.

Reads publish_ota_config.json (gitignored, NOT committed - see
publish_ota_config.example.json for the shape) for:
  - which PlatformIO environment maps to which registry.json token and
    remote firmware filename (kept OUT of this script, and out of git,
    since a token is effectively a bearer credential for that device's OTA)
  - the OTA server's SSH host and ota-server directory

Silently does nothing (just prints a note) if the config file is missing -
so a normal build for someone who hasn't set this up still works exactly
as before.

If a profile also has a "device_ip" set (the device's LAN address, when
it's reachable at build time - e.g. sitting on the bench on WiFi), this
also calls that device's own /api/control?cmd=OTA_TOKEN=<this profile's
token> right after publishing - so switching which vehicle you're building
for (VAG vs PSA env) is really just "pick the environment and build",
nothing else to remember by hand. Best-effort only: if the device isn't
reachable (out driving, different network, device_ip not set), this is
skipped with a clear message - it never fails the build.

The build timestamp is read back out of the just-built firmware.bin itself
(searched as a "Mmm d(d) yyyy HH:MM:SS" byte pattern) rather than computed
here in Python at publish time - those two clocks are two separate process
invocations a few seconds apart, and target_build must be byte-identical
to what the device will later report (FW_BUILD_STR = __DATE__ " " __TIME__,
baked in at compile time), or every future is_update_needed() comparison
against it is subtly wrong.
"""
Import("env")  # noqa: F821

import json
import os
import re
import subprocess
import urllib.error
import urllib.request

# PlatformIO execs this script via SConscript's exec(compile(...)), which
# does not define __file__ - use the project dir SCons already knows
# instead (same directory as this script and platformio.ini).
CONFIG_PATH = os.path.join(env["PROJECT_DIR"], "publish_ota_config.json")  # noqa: F821

_BUILD_PATTERN = re.compile(
    rb"[A-Z][a-z]{2} [ 0-9][0-9] 20\d{2} \d{2}:\d{2}:\d{2}"
)


def _load_config():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8-sig") as f:
            return json.load(f)
    except OSError:
        return None


def _extract_build_str(fw_bin_path):
    with open(fw_bin_path, "rb") as f:
        data = f.read()
    m = _BUILD_PATTERN.search(data)
    return m.group(0).decode("ascii") if m else None


def _ssh_access_ok(host):
    """True only if key-based SSH auth to host already works, with no
    prompt. BatchMode=yes makes ssh fail immediately (instead of hanging on
    a password prompt PlatformIO's build has no way to answer) if key auth
    isn't set up - exactly the case this function exists to catch and
    report clearly, instead of the build just hanging or scp failing with
    a cryptic error further down."""
    result = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, "true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def publish(source, target, env):  # noqa: ARG001
    config = _load_config()
    if config is None:
        print(f"[publish_ota] no {os.path.basename(CONFIG_PATH)} - skipping auto-publish "
              f"(see publish_ota_config.example.json)")
        return

    pioenv = env["PIOENV"]
    profile = config.get("profiles", {}).get(pioenv)
    if not profile:
        print(f"[publish_ota] no profile configured for env '{pioenv}' - skipping")
        return

    fw_bin = str(target[0])
    build_str = _extract_build_str(fw_bin)
    if not build_str:
        print("[publish_ota] WARNING: could not find a build timestamp in the "
              "compiled binary - not publishing (would corrupt target_build)")
        return

    host = config["ota_host"]
    ota_dir = config["ota_dir"]
    remote_path = f"{ota_dir}/firmware/{profile['remote_name']}"

    if not _ssh_access_ok(host):
        print(f"[publish_ota] WARNING: SSH access to {host} is not set up (no working "
              f"key-based login) - build succeeded but firmware was NOT published, "
              f"target_build NOT updated. Set up a key (e.g. ssh-copy-id {host}) and "
              f"rebuild to publish.")
        return

    print(f"[publish_ota] {pioenv}: build={build_str!r} -> {host}:{remote_path}")

    scp = subprocess.run(["scp", fw_bin, f"{host}:{remote_path}"])
    if scp.returncode != 0:
        print(f"[publish_ota] scp failed (exit {scp.returncode}) - target_build NOT updated")
        return

    remote_cmd = (
        f"cd {ota_dir} && python3 publish_receiver.py "
        f"{profile['token']} '{build_str}'"
    )
    receiver = subprocess.run(["ssh", host, remote_cmd])
    if receiver.returncode != 0:
        print(f"[publish_ota] publish_receiver.py failed (exit {receiver.returncode}) - "
              f"binary is on the server but target_build/push-check may not have run")
        return

    print(f"[publish_ota] done: {profile['remote_name']} published, "
          f"target_build set, push-check triggered")

    device_ip = profile.get("device_ip")
    if device_ip:
        _provision_device_token(device_ip, profile["token"], pioenv)


def _provision_device_token(device_ip, token, pioenv):
    """Best-effort: point the device (if it's reachable right now on this
    LAN) at this profile's own token, so building a different environment
    is the ONLY step needed when switching which vehicle the device is
    currently wired to - no separate manual OTA_TOKEN= call to remember."""
    url = f"http://{device_ip}/api/control?cmd=OTA_TOKEN={token}"
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            body = r.read().decode("utf-8", "replace").strip()
        if body == "OK":
            print(f"[publish_ota] device at {device_ip} switched to the {pioenv} token")
        else:
            print(f"[publish_ota] device at {device_ip} responded {body!r} "
                  f"(expected OK) - OTA_TOKEN may not be set, check manually")
    except (urllib.error.URLError, OSError) as e:
        print(f"[publish_ota] device at {device_ip} not reachable ({e}) - "
              f"OTA_TOKEN not updated on it. Fine if it's not on this LAN right "
              f"now (e.g. out driving); set it manually when it's back if needed.")


env.AddPostAction("$BUILD_DIR/firmware.bin", publish)  # noqa: F821

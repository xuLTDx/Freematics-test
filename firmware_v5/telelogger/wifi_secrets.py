"""PlatformIO pre-build script: prompt for WiFi passwords instead of storing
them in any tracked file, and ask what this specific build is for.

Resolution order per secret:
  1. Environment variable (WIFI_PWD / WIFI_PWD2) - lets CI/non-interactive
     builds set it without a prompt.
  2. Interactive prompt (only when stdin is a real terminal) via getpass,
     which does not echo the input.
  3. Empty string (e.g. background IntelliSense rebuilds, which run without
     a tty) - the device then relies on runtime NVS provisioning instead.

Nothing entered here is written to disk; it only lives in the compiled
.bin for this one local build.

Also prompts (same tty-only rule) for the build's purpose - ODO_READ
(normal driving/logging build) or CAN_SNIFF (bench-test build for capturing
raw CAN traffic alongside VCDS). The two must not run at the same time: the
odometer block's own AT-command traffic on the shared ELM327 link would
interrupt an active CAN sniff (ATM1) stream. A CAN_SNIFF build therefore
forces OBD polling off and sniffing on at boot regardless of NVS/HTTP
toggles - see BUILD_CAN_SNIFF in config.h / telelogger.ino.
"""

Import("env")  # noqa: F821

import getpass
import os
import sys


def _inject_secret(env_var_name, macro_name, prompt_label):
    value = os.environ.get(env_var_name)
    if value is None:
        if sys.stdin.isatty():
            try:
                value = getpass.getpass(f"{prompt_label} (Enter to skip): ")
            except (EOFError, KeyboardInterrupt):
                value = ""
        else:
            value = ""
    env.Append(CPPDEFINES=[(macro_name, env.StringifyMacro(value))])


def _inject_build_purpose():
    env_value = os.environ.get("BUILD_PURPOSE")
    is_can_sniff = False
    if env_value is not None:
        is_can_sniff = env_value.strip().upper() == "CAN_SNIFF"
    elif sys.stdin.isatty():
        try:
            # Deliberately reusing getpass.getpass() here even though "1"/"2"
            # isn't a secret: on Windows it writes its prompt straight to the
            # console via msvcrt, bypassing stdout entirely, which is the only
            # thing that reliably shows up under some build-task runners (e.g.
            # PlatformIO run via a VS Code task) - plain print()/input() go
            # through regular (sometimes fully buffered, even with flush=True
            # from this side) stdout and can render invisible there. The only
            # cost is the answer isn't echoed while typing, which is harmless
            # for a single digit.
            answer = getpass.getpass(
                "Build purpose - [1] ODO_READ (normal, default) "
                "or [2] CAN_SNIFF (bench test): "
            ).strip()
        except (EOFError, KeyboardInterrupt):
            answer = ""
        is_can_sniff = answer == "2"
    env.Append(CPPDEFINES=[("BUILD_CAN_SNIFF", 1 if is_can_sniff else 0)])
    print(f"[wifi_secrets] Build purpose: {'CAN_SNIFF' if is_can_sniff else 'ODO_READ'}", flush=True)


_inject_secret("WIFI_PWD", "WIFI_PASSWORD", "WiFi password (primary network)")
_inject_secret("WIFI_PWD2", "WIFI_PASSWORD2", "WiFi password (secondary network)")
_inject_build_purpose()

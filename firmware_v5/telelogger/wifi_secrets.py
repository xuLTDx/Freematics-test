"""PlatformIO pre-build script: prompt for WiFi passwords instead of storing
them in any tracked file.

Resolution order per secret:
  1. Environment variable (WIFI_PWD / WIFI_PWD2) - lets CI/non-interactive
     builds set it without a prompt.
  2. Interactive prompt (only when stdin is a real terminal) via getpass,
     which does not echo the input.
  3. Empty string (e.g. background IntelliSense rebuilds, which run without
     a tty) - the device then relies on runtime NVS provisioning instead.

Nothing entered here is written to disk; it only lives in the compiled
.bin for this one local build.
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


_inject_secret("WIFI_PWD", "WIFI_PASSWORD", "WiFi password (primary network)")
_inject_secret("WIFI_PWD2", "WIFI_PASSWORD2", "WiFi password (secondary network)")

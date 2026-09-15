"""PlatformIO pre-build script: prompt for WiFi passwords instead of storing
them in any tracked file, and ask what this specific build is for.

Resolution order per secret:
  0. A local secrets file OUTSIDE this repo (see SECRETS_FILE_PATH below) -
     lets a non-interactive build (e.g. this Claude Code session, which has
     no tty and previously could never get past step 2) supply real values
     without a prompt and without ever putting them in a tracked file.
  1. Environment variable (WIFI_PWD / WIFI_PWD2) - lets CI/non-interactive
     builds set it without a prompt.
  2. Interactive prompt (only when stdin is a real terminal) via getpass,
     which does not echo the input.
  3. Empty string (e.g. background IntelliSense rebuilds, which run without
     a tty) - the device then relies on runtime NVS provisioning instead.

Nothing entered here is written to disk by this script; the secrets file is
edited directly by the user and only lives in the compiled .bin for this one
local build (same "nothing tracked" guarantee as the old getpass-only flow).

Always builds BUILD_CAN_SNIFF=0 (ODO_READ) - no prompt/choice for this
anymore. That macro's CAN-sniff bench-test mode (forces OBD polling off,
ATM1 sniffing on at boot) predates HexSniff; now that HexSniff exists as its
own dedicated, more capable sniffing tool (own WiFi web UI, SD rotation,
live filter/send - see HexSniff/firmware/), there's no live use case left
that needs this repo's own build to do it too, so asking every build wasn't
worth it. The BUILD_CAN_SNIFF macro/config.h and telelogger.ino's runtime
handling of it are untouched - only the interactive choice is gone.
"""

Import("env")  # noqa: F821

import getpass
import os
import sys

# Fixed path outside the repo - never anywhere under Desktop/GitHub/... or
# any other tracked location. (HexSniff solves the same "no secrets in a
# tracked file" problem differently - a gitignored wifi_local.ini INSIDE its
# own repo dir, already fully non-interactive - this file is specific to
# this repo, not shared with it.)
SECRETS_FILE_PATH = os.path.join(os.path.expanduser("~"), ".freematics_build_secrets")


def _load_secrets_file():
    """Exports KEY=VALUE lines from SECRETS_FILE_PATH into os.environ, only
    for keys not already set - a real environment variable (e.g. from CI)
    always wins over the file. Missing file or unreadable lines are silently
    skipped: this is a convenience layer on top of the existing env-var/
    getpass/empty fallback chain, not a required step.
    """
    try:
        with open(SECRETS_FILE_PATH, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip()
        # No "and value" check here on purpose: a present-but-empty line
        # (e.g. "WIFI_PWD2=", secondary network intentionally unset) must
        # still set os.environ to "" so _inject_secret() sees a real (empty)
        # value and skips its own getpass prompt - not seeing the key at all
        # is what actually falls through to that interactive prompt.
        if key and key not in os.environ:
            os.environ[key] = value


_load_secrets_file()


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
    # No prompt/choice anymore - see the module docstring for why. Always
    # ODO_READ; BUILD_CAN_SNIFF stays defined (as 0) since config.h /
    # telelogger.ino still reference the macro.
    env.Append(CPPDEFINES=[("BUILD_CAN_SNIFF", 0)])
    print("[wifi_secrets] Build purpose: ODO_READ", flush=True)


_inject_secret("WIFI_PWD", "WIFI_PASSWORD", "WiFi password (primary network)")
_inject_secret("WIFI_PWD2", "WIFI_PASSWORD2", "WiFi password (secondary network)")
_inject_build_purpose()

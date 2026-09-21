# PlatformIO pre-script: -mfix-esp32-psram-cache-issue (and its -strategy
# companion) in build_flags only reaches CCFLAGS - SCons's flag parser
# doesn't propagate -m* flags to LINKFLAGS automatically. Without it also
# being present at LINK time, the linker's multilib selection never picks
# the cache-erratum-safe xtensa-esp32-elf/lib/esp32-psram/{libc,libm}.a
# variant, silently linking the plain (unfixed) libc/libm instead - even
# though every .cpp.o was compiled with the flag. Confirmed via `pio run -v`
# 2026-09-21: the flag was present in every compile command but absent from
# the final firmware.elf link command.
Import("env")  # noqa: F821

env.Append(LINKFLAGS=[
    "-mfix-esp32-psram-cache-issue",
    "-mfix-esp32-psram-cache-strategy=dupldst",
])

#!/usr/bin/python
#
# Applies the debounce counter patch. This script, the debounce_counter.patch
# file and the configuration options in platformio.ini can all be removed once
# https://github.com/SignalK/SensESP/commit/4b170fd0cf1dbfbb2dea20a54c0443c8ac32955c
# lands in a SensESP release.
import subprocess

result = subprocess.run(["patch", "-d", ".pio/libdeps/esp32dev/SensESP/",
                        "-p1", "-f", "-s", "-i", "../../../../debounce-counter.patch"])
if result.returncode != 0:
    print("Couldn't apply the debounce counter patch, maybe it's already applied?")

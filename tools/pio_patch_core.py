# PlatformIO "extra script": runs before every build and makes sure the Teensy
# core inside PlatformIO's package folder has the "Serial + FFB Wheel" USB
# type (see tools/patch_teensy_core.py for what that means). It only changes
# files the first time; after that it just refreshes the descriptor.
Import("env")  # noqa: F821  (provided by PlatformIO)
import os
import subprocess

pkg = env.PioPlatform().get_package_dir("framework-arduinoteensy")  # noqa: F821
core = os.path.join(pkg, "cores", "teensy4")
script = os.path.join(env.subst("$PROJECT_DIR"), "tools", "patch_teensy_core.py")  # noqa: F821
subprocess.check_call([env.subst("$PYTHONEXE"), script, core])  # noqa: F821

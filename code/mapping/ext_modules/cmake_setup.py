#!/usr/bin/env python
import subprocess
import os

from setup import is_debug_enabled


def main():
    install_instruction = "install"
    if is_debug_enabled():
        install_instruction = "develop"

    workingdir = os.path.dirname(os.path.abspath(__file__))
    subprocess.run(
        ["python", "setup.py", install_instruction, "--user"], cwd=workingdir
    )


if __name__ == "__main__":
    main()

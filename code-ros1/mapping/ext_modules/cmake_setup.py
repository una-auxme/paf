#!/usr/bin/env python
import subprocess
import os
import argparse
import sys

from setup import is_debug_enabled


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base_dir", required=True, type=str)
    parser.add_argument("--packages_dir", required=True, type=str)
    args = parser.parse_args()

    install_base_dir = args.base_dir
    install_lib_dir = args.packages_dir
    install_headers_dir = os.path.join(install_base_dir, "include")
    install_scripts_dir = os.path.join(install_base_dir, "scripts")
    install_data_dir = os.path.join(install_base_dir, "data")
    workingdir = os.path.dirname(os.path.abspath(__file__))

    args = [
        "install",
        "--install-base",
        install_base_dir,
        "--install-lib",
        install_lib_dir,
        "--install-headers",
        install_headers_dir,
        "--install-scripts",
        install_scripts_dir,
        "--install-data",
        install_data_dir,
    ]
    if is_debug_enabled():
        args = [
            "develop",
            "--prefix",
            install_base_dir,
            "--install-dir",
            install_lib_dir,
            "--script-dir",
            install_scripts_dir,
        ]

    subprocess.run(
        [sys.executable, "setup.py"] + args,
        cwd=workingdir,
        check=True,
    )


if __name__ == "__main__":
    main()

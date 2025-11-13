#!/bin/bash
set -e

mkdir ~/git
git clone https://github.com/una-auxme/paf.git ~/git/paf
cd ~/git/paf
./setup_env.sh

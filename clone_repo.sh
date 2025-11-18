#!/bin/bash
set -e

mkdir -p ~/git
if [ ! -d ~/git/paf ]; then
  git clone https://github.com/una-auxme/paf.git ~/git/paf
fi
cd ~/git/paf
./setup_env.sh

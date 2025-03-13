#!/bin/bash
set -e

cd /workspace

# Use this command to generate the base .rst files in the documentation.
sphinx-apidoc -o ./doc/sphinx/ ./code/mapping/ext_modules/mapping_common sphinx-apidoc --separate --force --module-first

# Rebuild mapping_common without compilation
echo "True" > ./code/mapping/ext_modules/.debug_enabled
cd /catkin_ws && catkin_make

# Build the markdown files
cd /workspace/doc/sphinx
sphinx-build -M markdown ./ ./_build

cp -r ./_build/markdown/* /workspace/doc/mapping

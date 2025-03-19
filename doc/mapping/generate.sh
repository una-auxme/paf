#!/bin/bash
# Execute this script with the /build/docker-compose.docs.yaml
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd "${SCRIPT_DIR}"

pydoc-markdown

if [ -d ./generated ]; then
    rm -r ./generated/
fi
mkdir -p ./generated/
cp -r ./.build/content/* ./generated/

shopt -s globstar
shopt -s nullglob
for file in ./generated/**/*.md; do
    sed '1 i\<!-- markdownlint-disable -->' -i "${file}"
    echo Disabled linter for "${file}"
done

#!/bin/bash

set -e
SCRIPT_DIR="$(realpath "$( dirname "${BASH_SOURCE[0]}" )")"

set -x
python "$SCRIPT_DIR/main.py" --derive all --codegen --outputdir /tmp/pyekf_output
mv /tmp/pyekf_output/ekf_defines.h "$SCRIPT_DIR/.."

#!/usr/bin/env bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/esl_sim"

make clean
make

cd "$SCRIPT_DIR"

eval '"$SCRIPT_DIR/esl_sim/px4_quad_sim"'